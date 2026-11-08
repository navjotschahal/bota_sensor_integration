#!/usr/bin/python3
"""Live comparison of the Bota sensor against the APCF joint-grid contact estimator.

No TF between sensor and robot is needed: everything here is frame-independent
(magnitudes, timing, angles between pushes) or lives in the robot base frame
(APCF's own contact point).

Outputs
  /bota_compare/markers  visualization_msgs/MarkerArray in ~base_frame, add it to
                         the APCF RViz: sensor-force arrow (sensor axes) on a gauge
                         beside the robot, |F| bars sensor vs APCF, a sphere at the
                         APCF contact point sized by the sensor |F|, and a text HUD.
  matplotlib window      |F| sensor vs APCF with contact shading; detector statistic
                         vs threshold and existence; sensor force components.
  <out_dir>/samples.csv  one row per APCF message with the time-matched sensor frame
  <out_dir>/sensor.csv   raw sensor stream (~1 kHz) if ~record_sensor_raw
  <out_dir>/pushes.csv   one row per push: means, ratio, latencies, angle test
  console                per-push summary and the pairwise-angle / Procrustes test

Push segmentation uses the sensor only: |F| > ~thr_on (N) starts a push, < ~thr_off
ends it, shorter than ~min_push_s is ignored. Keep the sensor orientation fixed
across pushes for the angle test to be meaningful.

Params: ~base_frame (iiwa_link_0), ~wrench_topic (/bota/wrench), ~contacts_topic
(/apcf/contacts), ~in_contact_topic (/apcf/in_contact), ~status_topic (/apcf/status),
~out_dir (~/bota_tests/<stamp>), ~record_sensor_raw (true), ~plot (true),
~window_s (15), ~thr_on (1.0), ~thr_off (0.5), ~min_push_s (0.15),
~gauge_xyz ([0.6, -0.7, 0.3]), ~scale_m_per_n (0.02)
"""
import csv
import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import Point, WrenchStamped
from std_msgs.msg import Bool, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

CONTACT_COLS = 12  # track_id existence link_index px py pz fx fy fz mass spread_m position_crb_m


def kabsch(src, dst, w):
    H = (np.asarray(src) * np.asarray(w)[:, None]).T @ np.asarray(dst)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    return Vt.T @ np.diag([1.0, 1.0, d]) @ U.T


def ang(a, b):
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return float("nan")
    return math.degrees(math.acos(float(np.clip(a @ b / (na * nb), -1.0, 1.0))))


class Compare:
    def __init__(self):
        g = rospy.get_param
        self.base_frame = g("~base_frame", "iiwa_link_0")
        self.window = float(g("~window_s", 15.0))
        self.thr_on = float(g("~thr_on", 1.0))
        self.thr_off = float(g("~thr_off", 0.5))
        self.min_push = float(g("~min_push_s", 0.15))
        self.gauge = [float(v) for v in g("~gauge_xyz", [0.6, -0.7, 0.3])]
        self.scale = float(g("~scale_m_per_n", 0.02))
        self.plot = bool(g("~plot", True))
        self.record_raw = bool(g("~record_sensor_raw", True))
        out = g("~out_dir", "")
        if not out:
            out = os.path.expanduser("~/bota_tests/%s" % time.strftime("%Y%m%d_%H%M%S"))
        self.out_dir = os.path.expanduser(out)
        os.makedirs(self.out_dir, exist_ok=True)

        self.lock = threading.Lock()
        n = int(self.window * 1100)
        self.s_t = deque(maxlen=n)      # sensor time
        self.s_f = deque(maxlen=n)      # sensor force (3,)
        self.s_n = deque(maxlen=n)      # |F|
        m = int(self.window * 300)
        self.e_t = deque(maxlen=m)      # apcf time
        self.e_f = deque(maxlen=m)      # apcf force (3,) of the strongest live track
        self.e_n = deque(maxlen=m)
        self.e_p = deque(maxlen=m)      # apcf point (3,)
        self.e_link = deque(maxlen=m)
        self.e_ex = deque(maxlen=m)     # existence
        self.c_t = deque(maxlen=m)      # in_contact time
        self.c_v = deque(maxlen=m)
        self.st_t = deque(maxlen=m)     # status time
        self.st_stat = deque(maxlen=m)  # detector statistic
        self.st_thr = deque(maxlen=m)   # detector threshold

        # push segmentation
        self.in_push = False
        self.push_t0 = None
        self.push_s = []
        self.push_e = []
        self.pushes = []   # dicts
        self.last_summary = ""

        # files
        self.f_samples = open(os.path.join(self.out_dir, "samples.csv"), "w", newline="")
        self.w_samples = csv.writer(self.f_samples)
        self.w_samples.writerow(["t", "sensor_Fn", "sensor_fx", "sensor_fy", "sensor_fz",
                                 "apcf_fn", "apcf_fx", "apcf_fy", "apcf_fz", "apcf_px", "apcf_py", "apcf_pz",
                                 "apcf_link", "apcf_existence", "apcf_n_tracks", "in_contact",
                                 "detector_stat", "detector_thr"])
        self.f_push = open(os.path.join(self.out_dir, "pushes.csv"), "w", newline="")
        self.w_push = csv.writer(self.f_push)
        self.w_push.writerow(["push", "t_on", "dur_s", "sensor_Fn_mean", "apcf_fn_mean", "ratio_apcf_over_sensor",
                              "latency_on_ms", "latency_off_ms", "apcf_link_mode", "apcf_existence_mean",
                              "angle_err_vs_prev_deg_mean", "procrustes_residual_deg"])
        self.f_raw = None
        if self.record_raw:
            self.f_raw = open(os.path.join(self.out_dir, "sensor.csv"), "w", newline="")
            self.w_raw = csv.writer(self.f_raw)
            self.w_raw.writerow(["t", "fx", "fy", "fz", "tx", "ty", "tz"])

        self.pub = rospy.Publisher("~markers", MarkerArray, queue_size=2)
        rospy.Subscriber(g("~wrench_topic", "/bota/wrench"), WrenchStamped, self.on_wrench, queue_size=50)
        rospy.Subscriber(g("~contacts_topic", "/apcf/contacts"), Float64MultiArray, self.on_contacts, queue_size=10)
        rospy.Subscriber(g("~in_contact_topic", "/apcf/in_contact"), Bool, self.on_in_contact, queue_size=10)
        rospy.Subscriber(g("~status_topic", "/apcf/status"), Float64MultiArray, self.on_status, queue_size=10)
        rospy.loginfo("bota_apcf_compare: base_frame=%s out_dir=%s thr_on=%.1fN", self.base_frame, self.out_dir, self.thr_on)

    # ---------------- callbacks ----------------
    def on_wrench(self, m):
        t = m.header.stamp.to_sec()
        f = np.array([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z])
        n = float(np.linalg.norm(f))
        with self.lock:
            self.s_t.append(t); self.s_f.append(f); self.s_n.append(n)
            if self.f_raw:
                self.w_raw.writerow([f"{t:.6f}", f"{f[0]:.4f}", f"{f[1]:.4f}", f"{f[2]:.4f}",
                                     f"{m.wrench.torque.x:.5f}", f"{m.wrench.torque.y:.5f}", f"{m.wrench.torque.z:.5f}"])
            self.segment(t, f, n)

    def on_contacts(self, m):
        t = rospy.Time.now().to_sec()
        rows = np.array(m.data, float).reshape(-1, CONTACT_COLS) if len(m.data) else np.zeros((0, CONTACT_COLS))
        if rows.shape[0]:
            fn = np.linalg.norm(rows[:, 6:9], axis=1) * (rows[:, 1] > 0.5)
            r = rows[int(np.argmax(fn))]
            f, p, link, ex = r[6:9].copy(), r[3:6].copy(), int(r[2]), float(r[1])
        else:
            f, p, link, ex = np.zeros(3), np.full(3, np.nan), -1, 0.0
        with self.lock:
            self.e_t.append(t); self.e_f.append(f); self.e_n.append(float(np.linalg.norm(f)))
            self.e_p.append(p); self.e_link.append(link); self.e_ex.append(ex)
            sf, sn = self.nearest_sensor(t)
            ic = self.c_v[-1] if self.c_v else False
            ds, dt_ = (self.st_stat[-1], self.st_thr[-1]) if self.st_stat else (np.nan, np.nan)
            self.w_samples.writerow([f"{t:.6f}", f"{sn:.4f}", *[f"{v:.4f}" for v in sf],
                                     f"{np.linalg.norm(f):.4f}", *[f"{v:.4f}" for v in f], *[f"{v:.4f}" for v in p],
                                     link, f"{ex:.3f}", rows.shape[0], int(ic), f"{ds:.4f}", f"{dt_:.4f}"])
            if self.in_push and np.linalg.norm(f) > 0:
                self.push_e.append(f)

    def on_in_contact(self, m):
        with self.lock:
            self.c_t.append(rospy.Time.now().to_sec()); self.c_v.append(bool(m.data))

    def on_status(self, m):
        d = list(m.data)
        if len(d) < 2:
            return
        with self.lock:
            self.st_t.append(rospy.Time.now().to_sec()); self.st_stat.append(d[0]); self.st_thr.append(d[1])

    def nearest_sensor(self, t):
        if not self.s_t:
            return np.zeros(3), 0.0
        ts = np.fromiter(self.s_t, float)
        i = int(np.searchsorted(ts, t))
        i = min(max(i, 0), len(ts) - 1)
        return self.s_f[i], self.s_n[i]

    # ---------------- push segmentation + metrics (lock held) ----------------
    def segment(self, t, f, n):
        if not self.in_push and n > self.thr_on:
            self.in_push, self.push_t0, self.push_s, self.push_e = True, t, [f], []
        elif self.in_push:
            if n > self.thr_off:
                self.push_s.append(f)
            else:
                self.in_push = False
                self.finish_push(t)

    def edge_time(self, t_ref, rising, lo, hi):
        """Time of first in_contact edge (rising/falling) within [t_ref+lo, t_ref+hi], else nan."""
        prev = None
        for tc, v in zip(self.c_t, self.c_v):
            if prev is not None and tc >= t_ref + lo and tc <= t_ref + hi:
                if (rising and v and not prev) or ((not rising) and prev and not v):
                    return tc
            prev = v
        return float("nan")

    def finish_push(self, t_off):
        dur = t_off - self.push_t0
        if dur < self.min_push or len(self.push_s) < 10:
            return
        ms = np.mean(self.push_s, 0)
        me = np.mean(self.push_e, 0) if self.push_e else np.zeros(3)
        sn, en = float(np.linalg.norm(ms)), float(np.linalg.norm(me))
        t_on_c = self.edge_time(self.push_t0, True, -0.3, min(1.0, dur))
        t_off_c = self.edge_time(t_off, False, -0.3, 1.0)
        lat_on = (t_on_c - self.push_t0) * 1e3 if not math.isnan(t_on_c) else float("nan")
        lat_off = (t_off_c - t_off) * 1e3 if not math.isnan(t_off_c) else float("nan")
        links = [l for tt, l in zip(self.e_t, self.e_link) if self.push_t0 <= tt <= t_off and l >= 0]
        link_mode = max(set(links), key=links.count) if links else -1
        exs = [e for tt, e in zip(self.e_t, self.e_ex) if self.push_t0 <= tt <= t_off]
        ex_mean = float(np.mean(exs)) if exs else 0.0

        # frame-independent direction test against previous pushes
        errs = []
        for p in self.pushes:
            if p["en"] > self.thr_on and en > self.thr_on:
                errs.append(abs(ang(ms, p["ms"]) - ang(me, p["me"])))
        ang_err = float(np.mean(errs)) if errs else float("nan")
        good = [p for p in self.pushes if p["en"] > self.thr_on] + ([{"ms": ms, "me": me, "sn": sn, "en": en}] if en > self.thr_on else [])
        resid = float("nan")
        if len(good) >= 3:
            S = np.array([p["ms"] / p["sn"] for p in good]); E = np.array([p["me"] / p["en"] for p in good])
            R = kabsch(S, E, np.array([min(p["sn"], p["en"]) for p in good]))
            resid = float(np.mean([ang(R @ s, e) for s, e in zip(S, E)]))

        k = len(self.pushes) + 1
        rec = dict(k=k, t0=self.push_t0, dur=dur, ms=ms, me=me, sn=sn, en=en, ratio=(en / sn if sn > 0 else float("nan")),
                   lat_on=lat_on, lat_off=lat_off, link=link_mode, ex=ex_mean, ang_err=ang_err, resid=resid)
        self.pushes.append(rec)
        self.w_push.writerow([k, f"{self.push_t0:.3f}", f"{dur:.3f}", f"{sn:.3f}", f"{en:.3f}", f"{rec['ratio']:.3f}",
                              f"{lat_on:.1f}", f"{lat_off:.1f}", link_mode, f"{ex_mean:.3f}", f"{ang_err:.1f}", f"{resid:.1f}"])
        self.f_push.flush(); self.f_samples.flush()
        self.last_summary = ("push %d: %.2fs  Bota %.2f N  APCF %.2f N  ratio %.2f  lat on %.0f ms off %.0f ms  link %d  exist %.2f"
                             % (k, dur, sn, en, rec["ratio"], lat_on, lat_off, link_mode, ex_mean))
        extra = ""
        if not math.isnan(ang_err):
            extra += "  angle-pair err %.1f deg" % ang_err
        if not math.isnan(resid):
            extra += "  procrustes resid %.1f deg (%d pushes)" % (resid, len(good))
        rospy.loginfo(self.last_summary + extra)

    # ---------------- markers ----------------
    def marker(self, mid, mtype, rgba, lifetime=0.5):
        m = Marker()
        m.header.frame_id = self.base_frame
        m.header.stamp = rospy.Time.now()
        m.ns, m.id, m.type, m.action = "bota_compare", mid, mtype, Marker.ADD
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.pose.orientation.w = 1.0
        m.lifetime = rospy.Duration(lifetime)
        return m

    def publish_markers(self):
        with self.lock:
            if not self.s_f:
                return
            # display smoothing: mean of last ~30 ms
            k = min(len(self.s_f), 30)
            fs = np.mean(list(self.s_f)[-k:], 0)
            sn = float(np.linalg.norm(fs))
            en = self.e_n[-1] if self.e_n else 0.0
            p = self.e_p[-1] if self.e_p else np.full(3, np.nan)
            link = self.e_link[-1] if self.e_link else -1
            ex = self.e_ex[-1] if self.e_ex else 0.0
            ic = self.c_v[-1] if self.c_v else False
            summary = self.last_summary
        gx, gy, gz = self.gauge
        arr = MarkerArray()
        # 0: sensor force arrow in SENSOR axes, on the gauge
        a = self.marker(0, Marker.ARROW, (1.0, 0.25, 0.1, 0.95))
        a.points = [Point(gx, gy, gz), Point(gx + fs[0] * self.scale, gy + fs[1] * self.scale, gz + fs[2] * self.scale)]
        a.scale.x, a.scale.y, a.scale.z = 0.012, 0.03, 0.04
        if sn < 0.3:
            a.action = Marker.DELETE
        arr.markers.append(a)
        # 1,2: magnitude bars
        for mid, val, dx, rgba in ((1, sn, -0.06, (1.0, 0.25, 0.1, 0.8)), (2, en, 0.06, (0.2, 0.5, 1.0, 0.8))):
            b = self.marker(mid, Marker.CUBE, rgba)
            h = max(val * self.scale, 0.002)
            b.scale.x, b.scale.y, b.scale.z = 0.04, 0.04, h
            b.pose.position.x, b.pose.position.y, b.pose.position.z = gx + dx, gy + 0.25, gz - 0.3 + h / 2
            arr.markers.append(b)
        # 3: sphere at APCF contact point, radius ~ sensor |F|
        s = self.marker(3, Marker.SPHERE, (1.0, 0.25, 0.1, 0.45))
        if np.all(np.isfinite(p)) and sn > 0.3:
            r = 0.02 + 0.004 * sn
            s.scale.x = s.scale.y = s.scale.z = r
            s.pose.position.x, s.pose.position.y, s.pose.position.z = p
        else:
            s.action = Marker.DELETE
        arr.markers.append(s)
        # 4: HUD text
        t = self.marker(4, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0))
        t.scale.z = 0.045
        t.pose.position.x, t.pose.position.y, t.pose.position.z = gx, gy, gz + 0.3
        t.text = ("Bota |F| %5.2f N   APCF |f| %5.2f N   ratio %4.2f\nin_contact %s   link %d   existence %.2f\n%s"
                  % (sn, en, (en / sn if sn > 0.3 else float("nan")), "YES" if ic else "no", link, ex, summary))
        arr.markers.append(t)
        # 5: gauge label
        lbl = self.marker(5, Marker.TEXT_VIEW_FACING, (0.8, 0.8, 0.8, 0.9))
        lbl.scale.z = 0.03
        lbl.pose.position.x, lbl.pose.position.y, lbl.pose.position.z = gx, gy + 0.25, gz - 0.34
        lbl.text = "red: Bota   blue: APCF   (%.0f cm / 10 N)" % (self.scale * 1000)
        arr.markers.append(lbl)
        self.pub.publish(arr)

    # ---------------- live plot ----------------
    def run_plot(self):
        try:
            import matplotlib
        except ImportError:
            rospy.logerr("matplotlib not importable with %s; recording and markers continue without the plot. "
                         "Run via bota_apcf_compare.launch (uses /usr/bin/python3) or set plot:=false.", sys.executable)
            rospy.spin()
            return
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        plt.ion()
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
        try:  # matplotlib >= 3.4
            fig.canvas.manager.set_window_title("Bota sensor vs APCF joint-grid estimator")
        except AttributeError:  # matplotlib 3.1 on Ubuntu 20.04
            fig.canvas.set_window_title("Bota sensor vs APCF joint-grid estimator")
        l_s, = ax1.plot([], [], "r-", lw=1.2, label="Bota |F| [N]")
        l_e, = ax1.plot([], [], "b-", lw=1.2, label="APCF |f| [N]")
        ax1.axhline(self.thr_on, color="gray", ls=":", lw=0.8)
        ax1.set_ylabel("force [N]"); ax1.legend(loc="upper left"); ax1.grid(alpha=0.3)
        l_st, = ax2.plot([], [], "k-", lw=1.0, label="APCF detector statistic")
        l_th, = ax2.plot([], [], "m--", lw=1.0, label="threshold")
        ax2b = ax2.twinx()
        l_ex, = ax2b.plot([], [], "g-", lw=1.0, label="existence")
        ax2b.set_ylim(-0.05, 1.05); ax2b.set_ylabel("existence")
        ax2.set_ylabel("statistic"); ax2.legend(loc="upper left"); ax2b.legend(loc="upper right"); ax2.grid(alpha=0.3)
        l_fx, = ax3.plot([], [], lw=0.9, label="fx"); l_fy, = ax3.plot([], [], lw=0.9, label="fy"); l_fz, = ax3.plot([], [], lw=0.9, label="fz")
        ax3.set_ylabel("Bota force [N]"); ax3.set_xlabel("time [s]"); ax3.legend(loc="upper left"); ax3.grid(alpha=0.3)
        shades = []
        while not rospy.is_shutdown() and plt.fignum_exists(fig.number):
            with self.lock:
                if self.s_t:
                    now = self.s_t[-1]
                    ts = np.fromiter(self.s_t, float) - now
                    F = np.array(self.s_f); Fn = np.fromiter(self.s_n, float)
                    te = np.fromiter(self.e_t, float) - now if self.e_t else np.array([])
                    En = np.fromiter(self.e_n, float) if self.e_n else np.array([])
                    Ex = np.fromiter(self.e_ex, float) if self.e_ex else np.array([])
                    tst = np.fromiter(self.st_t, float) - now if self.st_t else np.array([])
                    St = np.fromiter(self.st_stat, float) if self.st_stat else np.array([])
                    Th = np.fromiter(self.st_thr, float) if self.st_thr else np.array([])
                    tc = np.fromiter(self.c_t, float) - now if self.c_t else np.array([])
                    Cv = np.fromiter(self.c_v, float) if self.c_v else np.array([])
                else:
                    ts = None
            if ts is not None:
                l_s.set_data(ts, Fn); l_e.set_data(te, En)
                l_st.set_data(tst, St); l_th.set_data(tst, Th); l_ex.set_data(te, Ex)
                l_fx.set_data(ts, F[:, 0]); l_fy.set_data(ts, F[:, 1]); l_fz.set_data(ts, F[:, 2])
                for s in shades:
                    s.remove()
                shades = []
                if len(tc) > 1:
                    on = None
                    for tt, v in zip(tc, Cv):
                        if v and on is None:
                            on = tt
                        elif not v and on is not None:
                            shades.append(ax1.axvspan(on, tt, color="b", alpha=0.08)); on = None
                    if on is not None:
                        shades.append(ax1.axvspan(on, 0.0, color="b", alpha=0.08))
                ax1.set_xlim(-self.window, 0.0)
                ymax = max(1.0, float(Fn.max()) if Fn.size else 1.0, float(En.max()) if En.size else 1.0)
                ax1.set_ylim(-0.05 * ymax, 1.15 * ymax)
                if St.size:
                    ax2.set_ylim(min(0.0, float(np.nanmin(St))), 1.15 * max(float(np.nanmax(St)), float(np.nanmax(Th)) if Th.size else 1.0, 1e-6))
                fmax = max(1.0, float(np.abs(F).max()) if F.size else 1.0)
                ax3.set_ylim(-1.1 * fmax, 1.1 * fmax)
                fig.canvas.draw_idle()
            plt.pause(0.1)

    def close(self):
        for fh in (self.f_samples, self.f_push, self.f_raw):
            if fh:
                fh.close()
        rospy.loginfo("bota_apcf_compare: wrote %s (%d pushes)", self.out_dir, len(self.pushes))


def main():
    rospy.init_node("bota_compare")
    c = Compare()
    rospy.Timer(rospy.Duration(1.0 / 30.0), lambda _e: c.publish_markers())
    try:
        if c.plot:
            c.run_plot()
        else:
            rospy.spin()
    finally:
        c.close()


if __name__ == "__main__":
    main()
