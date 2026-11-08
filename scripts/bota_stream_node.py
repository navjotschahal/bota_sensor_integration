#!/usr/bin/env python3
"""ROS 1 node: receives the forwarder's UDP frame stream and publishes it.

Published (under the node's private namespace, default node name "bota"):
  ~wrench       geometry_msgs/WrenchStamped   force [N], torque [Nm]
  ~imu          sensor_msgs/Imu               linear acc [m/s^2], angular rate [rad/s]
  ~temperature  sensor_msgs/Temperature       [deg C]
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
"""
import socket
import threading

import rospy
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Imu, Temperature
from std_srvs.srv import Trigger, TriggerResponse

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
