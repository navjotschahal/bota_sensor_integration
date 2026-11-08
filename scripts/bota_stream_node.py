#!/usr/bin/env python3
"""ROS 1 node: receives the forwarder's UDP frame stream and publishes it.

Published (under the node's private namespace, default node name "bota"):
  ~wrench       geometry_msgs/WrenchStamped   force [N], torque [Nm]
  ~imu          sensor_msgs/Imu               linear acc [m/s^2], angular rate [rad/s]
  ~temperature  sensor_msgs/Temperature       [deg C]
  ~force_marker visualization_msgs/Marker     RViz arrow of the measured force at the frame origin
  ~force_norm   std_msgs/Float64              |F| [N], for rqt_plot
  /diagnostics  rate, drops, status flags, driver state
Services:
  ~tare         std_srvs/Trigger  -> hardware tare through the driver (deactivate, tare, activate)

Parameters:
  ~data_port (50100)  ~control_host (127.0.0.1)  ~control_port (50101)
  ~frame_id (bota_ft_sensor)   TF frame of the published wrench
  ~stamp_source (receive|sensor) receive: host wall clock at reception (default);
                      sensor: receive time corrected by the sensor clock delta,
                      smoother spacing at the cost of a constant unknown offset
  ~publish_invalid (false)  publish frames whose status.invalid bit is set
  ~timeout (1.0)      seconds without frames before a diagnostics error is raised
  ~marker/enable (true)       publish the RViz arrow
  ~marker/frame_id (frame_id) frame the arrow is drawn in (TF chain to come: mocap probe)
  ~marker/rate (30.0)         Hz, RViz cannot use 1 kHz
  ~marker/filter_hz (8.0)     first-order low-pass on the displayed vector, 0 = off
  ~marker/scale (0.02)        metres of arrow per newton
  ~marker/min_force (0.3)     N, below this the arrow is hidden
  ~marker/negate (false)      draw the reaction (what the robot feels) instead of the measured force
  ~marker/show_torque (false) second arrow for torque, metres per Nm = marker/scale * 10
  ~marker/color ([1.0, 0.2, 0.1, 0.9]) RGBA of the force arrow
"""
import socket
import threading

import rospy
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Point, WrenchStamped
from sensor_msgs.msg import Imu, Temperature
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker

from bota_ft_sensor import stream_protocol as sp


class BotaStreamNode:
    def __init__(self):
        self.data_port = int(rospy.get_param("~data_port", sp.DEFAULT_DATA_PORT))
        self.control_host = rospy.get_param("~control_host", sp.DEFAULT_HOST)
        self.control_port = int(rospy.get_param("~control_port", sp.DEFAULT_CONTROL_PORT))
        self.frame_id = rospy.get_param("~frame_id", "bota_ft_sensor")
        self.stamp_source = rospy.get_param("~stamp_source", "receive")
        self.publish_invalid = bool(rospy.get_param("~publish_invalid", False))
        self.timeout = float(rospy.get_param("~timeout", 1.0))

        self.pub_wrench = rospy.Publisher("~wrench", WrenchStamped, queue_size=10)
        self.pub_imu = rospy.Publisher("~imu", Imu, queue_size=10)
        self.pub_temp = rospy.Publisher("~temperature", Temperature, queue_size=2)
        self.srv_tare = rospy.Service("~tare", Trigger, self.on_tare)

        # RViz force arrow (display only; the data topics above stay unfiltered)
        self.marker_enable = bool(rospy.get_param("~marker/enable", True))
        self.marker_frame = rospy.get_param("~marker/frame_id", self.frame_id)
        self.marker_period = 1.0 / max(float(rospy.get_param("~marker/rate", 30.0)), 1e-3)
        self.marker_filter_hz = float(rospy.get_param("~marker/filter_hz", 8.0))
        self.marker_scale = float(rospy.get_param("~marker/scale", 0.02))
        self.marker_min_force = float(rospy.get_param("~marker/min_force", 0.3))
        self.marker_negate = bool(rospy.get_param("~marker/negate", False))
        self.marker_show_torque = bool(rospy.get_param("~marker/show_torque", False))
        color = rospy.get_param("~marker/color", [1.0, 0.2, 0.1, 0.9])
        self.marker_color = [float(c) for c in color] if len(color) == 4 else [1.0, 0.2, 0.1, 0.9]
        self.pub_marker = rospy.Publisher("~force_marker", Marker, queue_size=2)
        self.pub_force_norm = rospy.Publisher("~force_norm", Float64, queue_size=2)
        self.marker_last_pub = 0.0
        self.marker_f = None   # filtered force for display
        self.marker_t = None   # filtered torque for display
        self.marker_last_t = None

        self.lock = threading.Lock()
        self.n_frames = 0
        self.n_lost = 0
        self.n_invalid = 0
        self.n_bad = 0
        self.prev_seq = None
        self.last_rx = None
        self.last_frame = None
        self.rate_window = []  # receive times for rate estimate
        self.sensor_offset_ns = None  # host_time_ns - sensor_time_us*1000, filtered

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.data_port))
        self.sock.settimeout(0.5)

        self.diag = diagnostic_updater.Updater()
        self.diag.setHardwareID("bota_ft_sensor")
        self.diag.add("stream", self.diagnostics)

        rospy.loginfo("bota_stream_node: listening udp/%d, control %s:%d, frame_id %s",
                      self.data_port, self.control_host, self.control_port, self.frame_id)

    # -- services ------------------------------------------------------------
    def on_tare(self, _req):
        try:
            r = sp.control_request("tare", self.control_host, self.control_port, timeout=10.0)
        except socket.timeout:
            return TriggerResponse(False, "forwarder did not answer (container running?)")
        except Exception as exc:
            return TriggerResponse(False, "control error: %s" % exc)
        return TriggerResponse(bool(r.get("ok")), str(r.get("msg", "")))

    # -- diagnostics ---------------------------------------------------------
    def diagnostics(self, stat):
        with self.lock:
            now = rospy.Time.now().to_sec()
            age = (now - self.last_rx) if self.last_rx is not None else None
            window = [t for t in self.rate_window if now - t < 1.0]
            self.rate_window = window
            rate = len(window)
            f = self.last_frame
        if age is None:
            stat.summary(DiagnosticStatus.ERROR, "no frames received yet")
        elif age > self.timeout:
            stat.summary(DiagnosticStatus.ERROR, "no frames for %.1f s" % age)
        elif f is not None and f.driver_state != sp.STATE_ACTIVE:
            stat.summary(DiagnosticStatus.WARN, "driver state %s" % sp.STATE_NAMES.get(f.driver_state, "?"))
        elif f is not None and (f.status & (sp.STATUS_INVALID | sp.STATUS_OVERRANGE)):
            stat.summary(DiagnosticStatus.WARN, "sensor flags %s" % sp.status_flags(f.status))
        else:
            stat.summary(DiagnosticStatus.OK, "streaming %d Hz" % rate)
        stat.add("rate_hz", rate)
        stat.add("frames", self.n_frames)
        stat.add("lost", self.n_lost)
        stat.add("invalid", self.n_invalid)
        stat.add("bad_datagrams", self.n_bad)
        if f is not None:
            stat.add("driver_state", sp.STATE_NAMES.get(f.driver_state, "?"))
            for k, v in sp.status_flags(f.status).items():
                stat.add("status." + k, v)
            stat.add("temperature_c", "%.1f" % f.temperature)
            stat.add("forward_latency_ms", "%.3f" % ((rospy.Time.now().to_nsec() - f.host_time_ns) / 1e6)
                     if self.last_rx is not None and age is not None and age < 1.0 else "n/a")
        return stat

    # -- RViz arrow ------------------------------------------------------------
    def _lowpass(self, prev, x, dt):
        if prev is None or self.marker_filter_hz <= 0.0 or dt <= 0.0:
            return list(x)
        a = dt / (dt + 1.0 / (2.0 * 3.141592653589793 * self.marker_filter_hz))
        return [p + a * (v - p) for p, v in zip(prev, x)]

    def _arrow(self, stamp, mid, vec, scale, rgba):
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.marker_frame
        m.ns = "bota_force"
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.points = [Point(0.0, 0.0, 0.0), Point(vec[0] * scale, vec[1] * scale, vec[2] * scale)]
        m.scale.x = 0.012   # shaft diameter [m]
        m.scale.y = 0.03    # head diameter
        m.scale.z = 0.04    # head length
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.lifetime = rospy.Duration(0.5)   # vanishes when the stream stops
        m.pose.orientation.w = 1.0
        return m

    def update_marker(self, f, stamp):
        """Filter for display, publish |F| and the arrow at marker/rate."""
        t = stamp.to_sec()
        dt = (t - self.marker_last_t) if self.marker_last_t is not None else 0.0
        self.marker_last_t = t
        sign = -1.0 if self.marker_negate else 1.0
        self.marker_f = self._lowpass(self.marker_f, [sign * v for v in f.force], dt)
        if self.marker_show_torque:
            self.marker_t = self._lowpass(self.marker_t, [sign * v for v in f.torque], dt)
        if t - self.marker_last_pub < self.marker_period:
            return
        self.marker_last_pub = t
        fx, fy, fz = self.marker_f
        norm = (fx * fx + fy * fy + fz * fz) ** 0.5
        self.pub_force_norm.publish(Float64(norm))
        if not self.marker_enable or self.pub_marker.get_num_connections() == 0:
            return
        if norm < self.marker_min_force:
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.marker_frame
            m.ns, m.id, m.action = "bota_force", 0, Marker.DELETE
            self.pub_marker.publish(m)
        else:
            self.pub_marker.publish(self._arrow(stamp, 0, self.marker_f, self.marker_scale, self.marker_color))
        if self.marker_show_torque and self.marker_t is not None:
            self.pub_marker.publish(self._arrow(stamp, 1, self.marker_t, self.marker_scale * 10.0,
                                                [0.1, 0.4, 1.0, 0.9]))

    # -- main loop -----------------------------------------------------------
    def stamp_for(self, f, rx_ns):
        if self.stamp_source != "sensor":
            return rospy.Time(0, rx_ns)
        # first-order filter of the host/sensor clock offset; sensor clock wraps at 2^32 us (~71 min)
        obs = rx_ns - int(f.sensor_time_us) * 1000
        if self.sensor_offset_ns is None or abs(obs - self.sensor_offset_ns) > 2_000_000_000:
            self.sensor_offset_ns = obs
        else:
            self.sensor_offset_ns += int(0.01 * (obs - self.sensor_offset_ns))
        return rospy.Time(0, int(f.sensor_time_us) * 1000 + self.sensor_offset_ns)

    def spin(self):
        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(1024)
            except socket.timeout:
                self.diag.update()
                continue
            except OSError:
                break
            rx = rospy.Time.now()
            try:
                f = sp.unpack_frame(data)
            except ValueError as exc:
                with self.lock:
                    self.n_bad += 1
                rospy.logwarn_throttle(5.0, "bota_stream_node: %s", exc)
                continue

            with self.lock:
                self.n_lost += sp.seq_gap(self.prev_seq, f.seq)
                self.prev_seq = f.seq
                self.n_frames += 1
                self.last_rx = rx.to_sec()
                self.last_frame = f
                self.rate_window.append(self.last_rx)
                if f.status & sp.STATUS_INVALID:
                    self.n_invalid += 1
                    if not self.publish_invalid:
                        self.diag.update()
                        continue

            stamp = self.stamp_for(f, rx.to_nsec())

            w = WrenchStamped()
            w.header.stamp = stamp
            w.header.frame_id = self.frame_id
            w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = f.force
            w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z = f.torque
            self.pub_wrench.publish(w)
            self.update_marker(f, stamp)

            if self.pub_imu.get_num_connections() > 0:
                imu = Imu()
                imu.header = w.header
                imu.orientation_covariance[0] = -1.0  # no orientation estimate
                imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = f.acceleration
                imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = f.angular_rate
                self.pub_imu.publish(imu)

            if self.pub_temp.get_num_connections() > 0 and self.n_frames % 100 == 0:
                t = Temperature()
                t.header = w.header
                t.temperature = f.temperature
                self.pub_temp.publish(t)

            self.diag.update()


if __name__ == "__main__":
    rospy.init_node("bota")
    node = BotaStreamNode()
    try:
        node.spin()
    finally:
        node.sock.close()
