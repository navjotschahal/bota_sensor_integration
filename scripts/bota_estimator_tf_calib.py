#!/usr/bin/env python3
"""TEMPORARY sensor-to-robot TF from the contact estimator (no mocap).

Aligns the Bota sensor frame to the robot base by matching the measured force
(sensor frame) with the estimator's contact force (base frame) over several
pushes, then broadcasts base -> sensor_frame so both arrows show in one RViz
view. The translation follows the estimator's contact point while in contact,
so the sensor arrow is drawn at the estimated contact.

Caveats, by design:
  * Rotation is only valid while you hold the sensor in the same orientation
    as during the pushes. Re-run (or call ~reset) after changing your grip.
  * Direction agreement in the calibration directions is tautological; magnitude,
    timing and agreement in NEW push directions are the real tests.
  * This calibrates against the estimator under test. The mocap probe replaces it.

Procedure: estimator running, sensor tared. Push the robot firmly (> ~min_force)
in >= ~min_pushes clearly different directions, keeping the sensor orientation
fixed. Each push is averaged into one direction pair; after enough spread the
rotation is solved (Kabsch) and the TF starts. It keeps refining with further
pushes until ~freeze is called. ~reset clears everything.

Params: ~estimator (apcf|cpf), ~apcf_topic, ~cpf_topic, ~wrench_topic,
~base_frame (panda_link0), ~sensor_frame (bota_ft_sensor), ~min_force (4.0 N),
~min_pushes (3), ~min_angle_deg (20), ~sync_tol (0.05 s), ~negate (false),
~save (path to write the result yaml), ~load (yaml to publish instead of calibrating).
"""
import math
import threading

import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped, Vector3Stamped, WrenchStamped
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse


def kabsch(src, dst, w=None):
    """Rotation R minimizing sum w |dst - R src|^2 (rows are vectors)."""
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    w = np.ones(len(src)) if w is None else np.asarray(w, float)
    H = (src * w[:, None]).T @ dst
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    return Vt.T @ D @ U.T


def mat_to_quat(R):
    """Rotation matrix -> (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0] * 4
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


class EstimatorTfCalib:
    def __init__(self):
        self.estimator = rospy.get_param("~estimator", "apcf")
        self.base_frame = rospy.get_param("~base_frame", "panda_link0")
        self.sensor_frame = rospy.get_param("~sensor_frame", "bota_ft_sensor")
        self.min_force = float(rospy.get_param("~min_force", 4.0))
        self.min_pushes = int(rospy.get_param("~min_pushes", 3))
        self.min_angle = math.radians(float(rospy.get_param("~min_angle_deg", 20.0)))
        self.sync_tol = float(rospy.get_param("~sync_tol", 0.05))
        self.sign = -1.0 if rospy.get_param("~negate", False) else 1.0
        self.save_path = rospy.get_param("~save", "")
        load_path = rospy.get_param("~load", "")

        self.lock = threading.Lock()
        self.f_sensor = None      # (t, np.array)
        self.f_est = None         # (t, np.array)
        self.p_est = None         # latest estimator contact point (base frame)
        self.p_hold = np.zeros(3)
        self.push_s, self.push_e = [], []   # per-sample accumulators of the current push
        self.pairs = []            # list of (unit_sensor, unit_est, mag)
        self.R = None
        self.frozen = False
        self.residual_deg = None

        if load_path:
            d = yaml.safe_load(open(load_path))
            self.R = np.array(d["rotation_matrix"], float)
            self.frozen = True
            rospy.loginfo("loaded rotation from %s (residual %.1f deg); publishing only", load_path, d.get("residual_deg", -1))

        self.br = tf2_ros.TransformBroadcaster()
        rospy.Subscriber(rospy.get_param("~wrench_topic", "/bota/wrench"), WrenchStamped, self.on_wrench, queue_size=10)
        if self.estimator == "apcf":
            rospy.Subscriber(rospy.get_param("~apcf_topic", "apcf/contacts"), Float64MultiArray, self.on_apcf, queue_size=5)
        else:
            rospy.Subscriber(rospy.get_param("~cpf_topic", "cpf/contact_force"), Vector3Stamped, self.on_cpf, queue_size=5)
        rospy.Service("~freeze", Trigger, self.on_freeze)
        rospy.Service("~reset", Trigger, self.on_reset)
        rospy.loginfo("estimator=%s base=%s sensor=%s min_force=%.1fN pushes>=%d",
                      self.estimator, self.base_frame, self.sensor_frame, self.min_force, self.min_pushes)

    # -- inputs ---------------------------------------------------------------
    def on_wrench(self, m):
        f = self.sign * np.array([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z])
        with self.lock:
            self.f_sensor = (m.header.stamp.to_sec(), f)
        self.step()

    def on_cpf(self, m):
        with self.lock:
            self.f_est = (m.header.stamp.to_sec(), np.array([m.vector.x, m.vector.y, m.vector.z]))

    def on_apcf(self, m):
        # rows: track_id existence link_index px py pz fx fy fz mass spread position_crb
        cols = m.layout.dim[1].size if len(m.layout.dim) > 1 else 12
        rows = np.array(m.data, float).reshape(-1, cols) if len(m.data) else np.zeros((0, cols))
        with self.lock:
            if rows.shape[0] == 0:
                self.f_est = (rospy.Time.now().to_sec(), np.zeros(3))
                return
            best = rows[np.argmax(np.linalg.norm(rows[:, 6:9], axis=1) * (rows[:, 1] > 0.5))]
            self.f_est = (rospy.Time.now().to_sec(), best[6:9].copy())
            self.p_est = best[3:6].copy()

    # -- push segmentation + solve --------------------------------------------
    def step(self):
        with self.lock:
            if self.f_sensor is None or self.f_est is None:
                return
            ts, fs = self.f_sensor
            te, fe = self.f_est
            synced = abs(ts - te) <= self.sync_tol
            in_push = np.linalg.norm(fs) > self.min_force and np.linalg.norm(fe) > self.min_force and synced
            if in_push and not self.frozen:
                self.push_s.append(fs)
                self.push_e.append(fe)
                if self.p_est is not None:
                    self.p_hold = self.p_est
                return
            if self.push_s and not self.frozen:
                n = len(self.push_s)
                ms, me = np.mean(self.push_s, 0), np.mean(self.push_e, 0)
                self.push_s, self.push_e = [], []
                if n < 20:   # < ~20 ms of contact: ignore taps
                    return
                us, ue = ms / np.linalg.norm(ms), me / np.linalg.norm(me)
                if any(math.acos(np.clip(us @ p[0], -1, 1)) < self.min_angle for p in self.pairs):
                    rospy.loginfo("push %d samples, |Fs|=%.1f |Fe|=%.1f: too close to a previous direction, kept for refinement only",
                                  n, np.linalg.norm(ms), np.linalg.norm(me))
                else:
                    rospy.loginfo("push %d samples, |Fs|=%.1f |Fe|=%.1f: new direction %d/%d",
                                  n, np.linalg.norm(ms), np.linalg.norm(me), self.n_distinct() + 1, self.min_pushes)
                self.pairs.append((us, ue, min(np.linalg.norm(ms), np.linalg.norm(me))))
                self.solve()

    def n_distinct(self):
        reps = []
        for us, _, _ in self.pairs:
            if all(math.acos(np.clip(us @ r, -1, 1)) >= self.min_angle for r in reps):
                reps.append(us)
        return len(reps)

    def solve(self):
        if self.n_distinct() < max(2, self.min_pushes):
            return
        S = np.array([p[0] for p in self.pairs])
        E = np.array([p[1] for p in self.pairs])
        W = np.array([p[2] for p in self.pairs])
        R = kabsch(S, E, W)
        res = [math.degrees(math.acos(np.clip((R @ s) @ e, -1, 1))) for s, e in zip(S, E)]
        self.R = R
        self.residual_deg = float(np.mean(res))
        rospy.loginfo("solved base<-sensor rotation from %d pushes: mean residual %.1f deg (max %.1f). TF live.",
                      len(self.pairs), self.residual_deg, max(res))
        if self.save_path:
            self.save()

    def save(self):
        q = mat_to_quat(self.R)
        d = {"base_frame": self.base_frame, "sensor_frame": self.sensor_frame,
             "rotation_matrix": self.R.tolist(), "quaternion_xyzw": q,
             "residual_deg": self.residual_deg, "n_pushes": len(self.pairs),
             "note": "TEMPORARY: sensor orientation aligned to the contact estimator; valid for one grip only"}
        with open(self.save_path, "w") as fh:
            yaml.safe_dump(d, fh)
        rospy.loginfo("wrote %s", self.save_path)

    # -- services + TF ----------------------------------------------------------
    def on_freeze(self, _):
        self.frozen = True
        return TriggerResponse(self.R is not None, "frozen (rotation %s)" % ("set" if self.R is not None else "NOT solved yet"))

    def on_reset(self, _):
        with self.lock:
            self.pairs, self.push_s, self.push_e = [], [], []
            self.R, self.frozen, self.residual_deg = None, False, None
        return TriggerResponse(True, "reset; push again")

    def spin(self):
        rate = rospy.Rate(50)
        warned = False
        while not rospy.is_shutdown():
            with self.lock:
                R, p = self.R, self.p_hold.copy()
            if R is None:
                if not warned:
                    rospy.loginfo("waiting for pushes (need %d distinct directions above %.1f N)...", self.min_pushes, self.min_force)
                    warned = True
            else:
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.base_frame
                t.child_frame_id = self.sensor_frame
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p
                q = mat_to_quat(R)
                t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
                self.br.sendTransform(t)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("bota_estimator_tf_calib")
    EstimatorTfCalib().spin()
