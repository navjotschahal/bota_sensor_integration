#!/usr/bin/env python3
"""Sensor registry tool (ROS-free, Python 3.8+): which Bota sensor is plugged where.

  bota_sensors.py list                 registered sensors, present/absent, device, ports, container
  bota_sensors.py identify             USB serial devices not in the registry, as a yaml snippet
  bota_sensors.py udev [--install]     udev rules giving every sensor a stable /dev/bota/<name>
  bota_sensors.py env <name>           shell exports for the container script
  bota_sensors.py names                registered names, one per line
  bota_sensors.py get <name> <key>     one registry value (e.g. data_port)

Identity = USB (vendor, product, serial) of the adapter or dongle the sensor hangs on,
read from sysfs for every /dev/ttyACM* and /dev/ttyUSB*. Registry: config/sensors.yaml
(override with --registry or BOTA_SENSORS_YAML).
"""
import argparse
import glob
import json
import os
import subprocess
import sys

import yaml

_HERE = os.path.dirname(os.path.realpath(__file__))
DEFAULT_REGISTRY = os.environ.get(
    "BOTA_SENSORS_YAML", os.path.normpath(os.path.join(_HERE, "..", "config", "sensors.yaml")))
UDEV_RULES_PATH = "/etc/udev/rules.d/99-bota-sensors.rules"


def load_registry(path):
    with open(path) as fh:
        d = yaml.safe_load(fh) or {}
    sensors = d.get("sensors") or {}
    for name, s in sensors.items():
        s.setdefault("name", name)
    return sensors


def usb_identity(tty_path):
    """(vendor, product, serial, manufacturer, product_name) for /dev/ttyX from sysfs, or None."""
    name = os.path.basename(tty_path)
    dev = os.path.realpath("/sys/class/tty/%s/device" % name)
    d = dev
    while d not in ("/", "") and not os.path.isfile(os.path.join(d, "idVendor")):
        d = os.path.dirname(d)
    if not os.path.isfile(os.path.join(d, "idVendor")):
        return None

    def rd(f):
        try:
            with open(os.path.join(d, f)) as fh:
                return fh.read().strip()
        except OSError:
            return ""
    return {"vendor": rd("idVendor"), "product": rd("idProduct"), "serial": rd("serial"),
            "manufacturer": rd("manufacturer"), "product_name": rd("product"), "tty": tty_path}


def present_ttys():
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


def match(sensors):
    """Return {name: identity-or-None} and the list of unmatched identities."""
    ids = [i for i in (usb_identity(t) for t in present_ttys()) if i]
    found = {}
    for name, s in sensors.items():
        u = s.get("usb") or {}
        hit = None
        for i in ids:
            if (i["vendor"].lower() == str(u.get("vendor", "")).lower()
                    and i["product"].lower() == str(u.get("product", "")).lower()
                    and (not u.get("serial") or i["serial"] == str(u.get("serial")))):
                hit = i
                break
        found[name] = hit
    matched_ttys = {i["tty"] for i in found.values() if i}
    unmatched = [i for i in ids if i["tty"] not in matched_ttys]
    return found, unmatched


def container_state(name):
    try:
        out = subprocess.run(["docker", "ps", "-a", "--filter", "name=^bota_%s$" % name,
                              "--format", "{{.Status}}"], capture_output=True, text=True, timeout=5)
        return out.stdout.strip() or "-"
    except Exception:
        return "?"


def resolve_device(s, ident):
    """Host device node to pass into the container: udev symlink if present, else the live tty."""
    dev = s.get("device")
    if dev and os.path.exists(dev):
        return os.path.realpath(dev)
    if ident:
        return ident["tty"]
    return None


def cmd_list(sensors, args):
    found, unmatched = match(sensors)
    rows = []
    for name, s in sensors.items():
        i = found[name]
        dev = resolve_device(s, i)
        rows.append((name, s.get("model", "?"), s.get("generation", "?"),
                     "present" if i or (s.get("device") and os.path.exists(s["device"])) else "absent",
                     dev or "-", "%s/%s" % (s.get("data_port"), s.get("control_port")),
                     container_state(name)))
    w = [max(len(str(r[k])) for r in rows + [("name", "model", "gen", "state", "device", "data/ctrl", "container")]) for k in range(7)]
    fmt = "  ".join("%%-%ds" % x for x in w)
    print(fmt % ("name", "model", "gen", "state", "device", "data/ctrl", "container"))
    for r in rows:
        print(fmt % r)
    if unmatched:
        print("\nunregistered serial devices (see `identify`):")
        for i in unmatched:
            print("  %s  %s:%s serial=%s  %s %s" % (i["tty"], i["vendor"], i["product"], i["serial"],
                                                    i["manufacturer"], i["product_name"]))
    return 0


def cmd_identify(sensors, args):
    found, unmatched = match(sensors)
    if not unmatched:
        print("every connected serial device is registered")
        return 0
    print("# paste under `sensors:` in %s and fill model/config/ports" % args.registry)
    for n, i in enumerate(unmatched):
        guess = "usb-cdc" if i["vendor"].lower() == "0483" else ("rs422-ftdi" if i["vendor"].lower() == "0403" else "serial")
        print("""  new_sensor_%d:
    description: %s %s (plugged at %s)
    model: FILL
    generation: genA | gen0
    link: %s
    usb: {vendor: "%s", product: "%s", serial: "%s"}
    device: /dev/bota/new_sensor_%d
    driver_config: FILL.json
    data_port: FILL
    control_port: FILL
    ros: {frame_id: new_sensor_%d_sensor}""" % (n, i["manufacturer"], i["product_name"], i["tty"], guess,
                                               i["vendor"], i["product"], i["serial"], n, n))
    return 0


def udev_rules(sensors):
    lines = ["# Generated by bota_sensors.py udev from sensors.yaml. Stable names for Bota sensor links.",
             "# Reload: sudo udevadm control --reload-rules && sudo udevadm trigger"]
    for name, s in sensors.items():
        u = s.get("usb")
        if not u or not s.get("device"):
            continue
        link = s["device"][len("/dev/"):] if s["device"].startswith("/dev/") else s["device"]
        cond = 'SUBSYSTEM=="tty", ATTRS{idVendor}=="%s", ATTRS{idProduct}=="%s"' % (u["vendor"], u["product"])
        if u.get("serial"):
            cond += ', ATTRS{serial}=="%s"' % u["serial"]
        lines.append('%s, SYMLINK+="%s", MODE="0666"   # %s %s' % (cond, link, name, s.get("model", "")))
    return "\n".join(lines) + "\n"


def cmd_udev(sensors, args):
    rules = udev_rules(sensors)
    if not args.install:
        print(rules, end="")
        print("# install with:  bota_sensors.py udev --install   (asks for sudo)", file=sys.stderr)
        return 0
    tmp = "/tmp/99-bota-sensors.rules"
    with open(tmp, "w") as fh:
        fh.write(rules)
    cmds = ["sudo install -m 644 %s %s" % (tmp, UDEV_RULES_PATH),
            "sudo udevadm control --reload-rules", "sudo udevadm trigger --subsystem-match=tty"]
    for c in cmds:
        print("+ " + c)
        if subprocess.call(c, shell=True) != 0:
            print("failed: " + c, file=sys.stderr)
            return 1
    print("installed %s; check: ls -l /dev/bota/" % UDEV_RULES_PATH)
    return 0


def cmd_env(sensors, args):
    s = sensors.get(args.name)
    if not s:
        print("unknown sensor %r; registered: %s" % (args.name, ", ".join(sensors)), file=sys.stderr)
        return 1
    found, _ = match({args.name: s})
    dev = resolve_device(s, found[args.name])
    out = {
        "BOTA_SENSOR_NAME": args.name,
        "BOTA_CONFIG": "/config/%s" % s["driver_config"],
        "BOTA_DATA_PORT": str(s["data_port"]),
        "BOTA_CONTROL_PORT": str(s["control_port"]),
        "BOTA_SERIAL_DEV": dev or "",
        "BOTA_HAS_SERIAL": "1" if s.get("usb") or s.get("device") else "0",
    }
    for k, v in out.items():
        print("export %s=%s" % (k, json.dumps(v)))
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--registry", default=DEFAULT_REGISTRY)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    sub.add_parser("identify")
    u = sub.add_parser("udev"); u.add_argument("--install", action="store_true")
    e = sub.add_parser("env"); e.add_argument("name")
    sub.add_parser("names")
    g = sub.add_parser("get"); g.add_argument("name"); g.add_argument("key")
    args = ap.parse_args()
    sensors = load_registry(args.registry)
    if args.cmd == "names":
        print("\n".join(sensors))
        return 0
    if args.cmd == "get":
        s = sensors.get(args.name)
        if not s:
            print("unknown sensor %r" % args.name, file=sys.stderr); return 1
        v = s
        for k in args.key.split("."):
            v = (v or {}).get(k)
        print("" if v is None else v)
        return 0
    return {"list": cmd_list, "identify": cmd_identify, "udev": cmd_udev, "env": cmd_env}[args.cmd](sensors, args)


if __name__ == "__main__":
    sys.exit(main())
