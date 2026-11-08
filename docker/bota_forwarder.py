#!/usr/bin/env python3
"""Runs inside the container. Owns the sensor through the official Bota driver
and forwards every frame as one UDP datagram (see bota_ft_sensor.stream_protocol).

Driver API used is the documented Python API of `bota-driver` (PyPI):
BotaDriver(config_path), configure(), tare(), activate(), deactivate(),
shutdown(), read_frame_blocking(), read_frame(), get_driver_state(),
get_expected_timestep_us(), get_driver_version_string(). tare() is only legal in
INACTIVE, so a runtime tare request deactivates, tares, and re-activates.

Environment:
    BOTA_CONFIG         path to the driver JSON config (required)
    BOTA_DATA_HOST      destination of the data stream (default 127.0.0.1)
    BOTA_DATA_PORT      destination UDP port (default 50100)
    BOTA_CONTROL_PORT   UDP port to listen for control requests (default 50101)
    BOTA_TARE_ON_START  "1" (default) to tare once before activation
    BOTA_READ_MODE      "blocking" (default, every frame) or "poll" (latest frame
                        at the sensor's expected timestep; use for polled links
                        such as CANopen over EtherCAT if blocking stalls)
"""
import json
import os
import signal
import socket
import sys
import threading
import time

import bota_driver

from bota_ft_sensor import stream_protocol as sp

LOG_PREFIX = "[bota_forwarder] "


def log(msg):
    sys.stderr.write(LOG_PREFIX + msg + "\n")
    sys.stderr.flush()


def state_code(driver):
    try:
        name = driver.get_driver_state().name
    except Exception:
        return sp.STATE_UNKNOWN
    return {
        "INITIAL": sp.STATE_UNCONFIGURED,
        "UNCONFIGURED": sp.STATE_UNCONFIGURED,
        "INACTIVE": sp.STATE_INACTIVE,
        "ACTIVE": sp.STATE_ACTIVE,
        "FINALIZED": sp.STATE_FINALIZED,
        "TERMINAL": sp.STATE_FINALIZED,
        "ERROR_PROCESSING": sp.STATE_ERROR,
    }.get(name, sp.STATE_TRANSITION)  # CONFIGURING, ACTIVATING, ... -> TRANSITION


def state_name(driver):
    try:
        return driver.get_driver_state().name
    except Exception:
        return "UNKNOWN"


class Forwarder:
    def __init__(self, config_path, data_host, data_port, control_port,
                 tare_on_start, read_mode):
        self.config_path = config_path
        self.data_addr = (data_host, data_port)
        self.control_port = control_port
        self.tare_on_start = tare_on_start
        self.read_mode = read_mode

        self.driver = None
        self.data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctrl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.ctrl_sock.bind(("0.0.0.0", control_port))
        self.ctrl_sock.settimeout(0.5)

        self.stop = threading.Event()
        self.lock = threading.Lock()        # guards driver lifecycle calls
        self.pause = threading.Event()      # ask the read loop to step aside
        self.paused = threading.Event()     # read loop acknowledges
        self.seq = 0
        self.frames = 0
        self.last_frame_time = None
        self.start_time = time.time()
        self.expected_dt = None

    # ---- lifecycle -------------------------------------------------------
    def start_driver(self):
        log("driver config: %s" % self.config_path)
        self.driver = bota_driver.BotaDriver(self.config_path)
        log("bota-driver version %s" % self.driver.get_driver_version_string())
        if not self.driver.configure():
            raise RuntimeError("configure() failed (state %s)" % state_name(self.driver))
        if self.tare_on_start:
            if not self.driver.tare():
                raise RuntimeError("tare() failed (state %s)" % state_name(self.driver))
            log("tared on start")
        if not self.driver.activate():
            raise RuntimeError("activate() failed (state %s)" % state_name(self.driver))
        self.expected_dt = self._expected_dt()
        if self.expected_dt:
            log("expected timestep %.6f s (%.1f Hz)" % (self.expected_dt, 1.0 / self.expected_dt))
        else:
            log("expected timestep unknown")
        log("ACTIVE, streaming to %s:%d, control on :%d, read mode %s"
            % (self.data_addr[0], self.data_addr[1], self.control_port, self.read_mode))

    def _expected_dt(self):
        """Seconds between frames. bota-driver 1.1.18 exposes get_expected_timestep_us();
        the docs describe get_expected_timestep() returning a timedelta. Accept both."""
        try:
            if hasattr(self.driver, "get_expected_timestep_us"):
                us = self.driver.get_expected_timestep_us()
                us = us.total_seconds() * 1e6 if hasattr(us, "total_seconds") else float(us)
                return us / 1e6 if us > 0 else None
            if hasattr(self.driver, "get_expected_timestep"):
                ts = self.driver.get_expected_timestep()
                return ts.total_seconds() if hasattr(ts, "total_seconds") else float(ts)
        except Exception as exc:
            log("expected timestep query failed: %s" % exc)
        return None

    def stop_driver(self):
        if self.driver is None:
            return
        try:
            if state_name(self.driver) == "ACTIVE" and not self.driver.deactivate():
                log("deactivate() failed")
            if not self.driver.shutdown():
                log("shutdown() failed")
        except Exception as exc:
            log("error during shutdown: %s" % exc)
        self.driver = None

    def do_tare(self):
        """Deactivate -> tare -> activate while the read loop is paused."""
        self.pause.set()
        if not self.paused.wait(timeout=max(2.0, 20 * (self.expected_dt or 0.1))):
            self.pause.clear()
            return False, "read loop did not pause (sensor not delivering frames?)"
        try:
            with self.lock:
                if not self.driver.deactivate():
                    return False, "deactivate() failed (state %s)" % state_name(self.driver)
                ok = self.driver.tare()
                if not self.driver.activate():
                    return False, "activate() failed after tare (state %s)" % state_name(self.driver)
                if not ok:
                    return False, "tare() failed"
                return True, "tared"
        finally:
            self.pause.clear()

    # ---- read loop -------------------------------------------------------
    def read_loop(self):
        poll = self.read_mode == "poll"
        period = self.expected_dt or 0.01
        next_t = time.monotonic()
        while not self.stop.is_set():
            if self.pause.is_set():
                self.paused.set()
                while self.pause.is_set() and not self.stop.is_set():
                    time.sleep(0.005)
                self.paused.clear()
                next_t = time.monotonic()
                continue
            try:
                with self.lock:
                    if poll:
                        frame = self.driver.read_frame()
                    else:
                        frame = self.driver.read_frame_blocking()
                    st = state_code(self.driver)
            except Exception as exc:
                log("read error: %s" % exc)
                time.sleep(0.1)
                continue
            now_ns = time.time_ns()
            try:
                status_val = int(frame.status.val)
            except Exception:
                status_val = 0
            pkt = sp.pack_frame(
                self.seq, st, now_ns, int(frame.timestamp), status_val,
                frame.force, frame.torque, frame.acceleration, frame.angular_rate,
                frame.temperature,
            )
            try:
                self.data_sock.sendto(pkt, self.data_addr)
            except OSError as exc:
                log("send error: %s" % exc)
            self.seq = (self.seq + 1) & 0xFFFF
            self.frames += 1
            self.last_frame_time = time.time()
            if poll:
                next_t += period
                delay = next_t - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_t = time.monotonic()

    # ---- control ---------------------------------------------------------
    def status_dict(self):
        uptime = time.time() - self.start_time
        return {
            "ok": True,
            "state": state_name(self.driver) if self.driver else "NONE",
            "driver_version": self.driver.get_driver_version_string() if self.driver else None,
            "frames": self.frames,
            "mean_rate_hz": self.frames / uptime if uptime > 0 else 0.0,
            "expected_dt_s": self.expected_dt,
            "last_frame_age_s": (time.time() - self.last_frame_time) if self.last_frame_time else None,
            "config": self.config_path,
            "protocol_version": sp.PROTOCOL_VERSION,
        }

    def control_loop(self):
        while not self.stop.is_set():
            try:
                data, addr = self.ctrl_sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                req = json.loads(data.decode("utf-8"))
                cmd = req.get("cmd")
            except Exception:
                reply = {"ok": False, "msg": "malformed request"}
            else:
                if cmd == "ping":
                    reply = {"ok": True, "msg": "pong"}
                elif cmd == "status":
                    reply = self.status_dict()
                elif cmd == "tare":
                    ok, msg = self.do_tare()
                    reply = {"ok": ok, "msg": msg}
                    log("tare request from %s: %s" % (addr[0], msg))
                else:
                    reply = {"ok": False, "msg": "unknown cmd %r" % (cmd,)}
            try:
                self.ctrl_sock.sendto(json.dumps(reply).encode("utf-8"), addr)
            except OSError:
                pass

    # ---- main ------------------------------------------------------------
    def run(self):
        self.start_driver()
        ctrl = threading.Thread(target=self.control_loop, name="control", daemon=True)
        ctrl.start()
        reader = threading.Thread(target=self.read_loop, name="reader", daemon=True)
        reader.start()
        last_report = time.time()
        last_frames = 0
        while not self.stop.is_set():
            time.sleep(0.5)
            now = time.time()
            if now - last_report >= 5.0:
                rate = (self.frames - last_frames) / (now - last_report)
                age = (now - self.last_frame_time) if self.last_frame_time else float("inf")
                if age > 2.0:
                    log("WARNING: no frames for %.1f s (state %s)" % (age, state_name(self.driver)))
                else:
                    log("%.1f Hz, %d frames total" % (rate, self.frames))
                last_report, last_frames = now, self.frames
        reader.join(timeout=2.0)
        self.stop_driver()
        log("exit")


def main():
    config = os.environ.get("BOTA_CONFIG")
    if not config:
        log("BOTA_CONFIG is not set")
        return 2
    if not os.path.isfile(config):
        log("config file not found: %s" % config)
        return 2
    fwd = Forwarder(
        config_path=config,
        data_host=os.environ.get("BOTA_DATA_HOST", sp.DEFAULT_HOST),
        data_port=int(os.environ.get("BOTA_DATA_PORT", sp.DEFAULT_DATA_PORT)),
        control_port=int(os.environ.get("BOTA_CONTROL_PORT", sp.DEFAULT_CONTROL_PORT)),
        tare_on_start=os.environ.get("BOTA_TARE_ON_START", "1") == "1",
        read_mode=os.environ.get("BOTA_READ_MODE", "blocking"),
    )

    def on_signal(signum, _frame):
        log("signal %d, stopping" % signum)
        fwd.stop.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        fwd.run()
    except Exception as exc:
        log("FATAL: %s" % exc)
        fwd.stop.set()
        fwd.stop_driver()
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
