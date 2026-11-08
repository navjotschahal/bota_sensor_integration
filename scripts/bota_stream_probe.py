#!/usr/bin/env python3
"""ROS-free check of the forwarder: query its control port and/or dump frames.

  bota_stream_probe.py --status            ask the forwarder for its status
  bota_stream_probe.py --tare              request a tare
  bota_stream_probe.py --listen 3          print frames for 3 seconds + rate
"""
import argparse
import socket
import sys
import time

from bota_ft_sensor import stream_protocol as sp


def listen(port, seconds):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(1.0)
    t0 = time.time()
    n = lost = 0
    prev = None
    last_print = 0.0
    while time.time() - t0 < seconds:
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            print("no datagrams on udp/%d ..." % port)
            continue
        try:
            f = sp.unpack_frame(data)
        except ValueError as exc:
            print("bad datagram: %s" % exc)
            continue
        lost += sp.seq_gap(prev, f.seq)
        prev = f.seq
        n += 1
        if time.time() - last_print > 0.5:
            last_print = time.time()
            lat_ms = (time.time_ns() - f.host_time_ns) / 1e6
            print("seq %5d state %-12s F=[%8.3f %8.3f %8.3f] N  T=[%7.4f %7.4f %7.4f] Nm  %5.1fC  flags=%s  lat=%.2fms"
                  % (f.seq, sp.STATE_NAMES.get(f.driver_state, "?"), *f.force, *f.torque,
                     f.temperature, sp.status_flags(f.status), lat_ms))
    dt = time.time() - t0
    print("%d frames in %.1fs = %.1f Hz, %d lost" % (n, dt, n / dt if dt else 0, lost))
    return 0 if n else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=sp.DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=None, help="control port (status/tare) or data port (listen)")
    ap.add_argument("--status", action="store_true")
    ap.add_argument("--tare", action="store_true")
    ap.add_argument("--listen", type=float, metavar="SECONDS")
    a = ap.parse_args()
    rc = 0
    if a.status or a.tare:
        port = a.port or sp.DEFAULT_CONTROL_PORT
        try:
            if a.status:
                print(sp.control_request("status", a.host, port))
            if a.tare:
                r = sp.control_request("tare", a.host, port, timeout=10.0)
                print(r)
                rc |= 0 if r.get("ok") else 1
        except socket.timeout:
            print("forwarder did not answer on %s:%d (container running?)" % (a.host, port))
            rc = 1
    if a.listen:
        rc |= listen(a.port or sp.DEFAULT_DATA_PORT, a.listen)
    if not (a.status or a.tare or a.listen):
        ap.print_help()
    return rc


if __name__ == "__main__":
    sys.exit(main())
