"""Wire format between the containerized Bota driver (forwarder) and clients.

One UDP datagram per sensor frame, little-endian, fixed size. Designed so any
client (ROS 1, ROS 2, plain C++) can parse it with a single struct unpack.
Must stay importable on Python 3.8 (host) and 3.12 (container): stdlib only.

Data datagram layout (DATA_FORMAT):
    4s   magic            b"BOTA"
    B    version          PROTOCOL_VERSION
    B    driver_state     DriverState code (see STATE_*)
    H    seq              frame counter, wraps at 65536 (drop detection)
    Q    host_time_ns     time.time_ns() on the forwarder when the frame was read
    I    sensor_time_us   sensor timestamp, microseconds since sensor boot
    H    status           raw 16-bit status word (bit0 throttled, bit1 overrange,
                          bit2 invalid, bit3 raw)
    H    reserved
    6f   fx fy fz tx ty tz   N, Nm
    3f   ax ay az            m/s^2
    3f   wx wy wz            rad/s
    f    temperature         deg C

Control channel: JSON request/response over UDP to the forwarder's control
port. Requests: {"cmd": "tare"}, {"cmd": "status"}, {"cmd": "ping"}.
Responses: {"ok": bool, "msg": str, ...}.
"""
import json
import socket
import struct
from collections import namedtuple

PROTOCOL_VERSION = 1
MAGIC = b"BOTA"

DEFAULT_DATA_PORT = 50100
DEFAULT_CONTROL_PORT = 50101
DEFAULT_HOST = "127.0.0.1"

DATA_FORMAT = "<4sBBHQIHH6f3f3ff"
DATA_SIZE = struct.calcsize(DATA_FORMAT)  # 76 bytes
_data_struct = struct.Struct(DATA_FORMAT)

# Driver lifecycle codes (mirror bota_driver.DriverState names).
STATE_UNKNOWN = 0
STATE_UNCONFIGURED = 1
STATE_INACTIVE = 2
STATE_ACTIVE = 3
STATE_FINALIZED = 4
STATE_TRANSITION = 5   # any CONFIGURING/ACTIVATING/... transition state
STATE_ERROR = 6
STATE_NAMES = {
    STATE_UNKNOWN: "UNKNOWN",
    STATE_UNCONFIGURED: "UNCONFIGURED",
    STATE_INACTIVE: "INACTIVE",
    STATE_ACTIVE: "ACTIVE",
    STATE_FINALIZED: "FINALIZED",
    STATE_TRANSITION: "TRANSITION",
    STATE_ERROR: "ERROR",
}

STATUS_THROTTLED = 1 << 0
STATUS_OVERRANGE = 1 << 1
STATUS_INVALID = 1 << 2
STATUS_RAW = 1 << 3

Frame = namedtuple(
    "Frame",
    [
        "version", "driver_state", "seq", "host_time_ns", "sensor_time_us",
        "status", "force", "torque", "acceleration", "angular_rate", "temperature",
    ],
)


def pack_frame(seq, driver_state, host_time_ns, sensor_time_us, status,
               force, torque, acceleration, angular_rate, temperature):
    """Serialize one frame. Sequences are 3-element iterables of floats."""
    return _data_struct.pack(
        MAGIC, PROTOCOL_VERSION, int(driver_state) & 0xFF, int(seq) & 0xFFFF,
        int(host_time_ns), int(sensor_time_us) & 0xFFFFFFFF, int(status) & 0xFFFF, 0,
        force[0], force[1], force[2], torque[0], torque[1], torque[2],
        acceleration[0], acceleration[1], acceleration[2],
        angular_rate[0], angular_rate[1], angular_rate[2],
        float(temperature),
    )


def unpack_frame(data):
    """Parse a datagram into a Frame. Raises ValueError on bad input."""
    if len(data) != DATA_SIZE:
        raise ValueError("bad datagram size %d (expected %d)" % (len(data), DATA_SIZE))
    f = _data_struct.unpack(data)
    if f[0] != MAGIC:
        raise ValueError("bad magic %r" % (f[0],))
    if f[1] != PROTOCOL_VERSION:
        raise ValueError("protocol version %d, expected %d" % (f[1], PROTOCOL_VERSION))
    return Frame(
        version=f[1], driver_state=f[2], seq=f[3], host_time_ns=f[4],
        sensor_time_us=f[5], status=f[6],
        force=f[8:11], torque=f[11:14], acceleration=f[14:17],
        angular_rate=f[17:20], temperature=f[20],
    )


def status_flags(status):
    """Return dict of the four documented status bits."""
    return {
        "throttled": bool(status & STATUS_THROTTLED),
        "overrange": bool(status & STATUS_OVERRANGE),
        "invalid": bool(status & STATUS_INVALID),
        "raw": bool(status & STATUS_RAW),
    }


def seq_gap(prev_seq, seq):
    """Number of frames lost between prev_seq and seq (0 when consecutive)."""
    if prev_seq is None:
        return 0
    return ((seq - prev_seq - 1) & 0xFFFF)


def control_request(cmd, host=DEFAULT_HOST, port=DEFAULT_CONTROL_PORT, timeout=3.0, **kwargs):
    """Send a JSON command to the forwarder control port and return its reply dict.

    Raises socket.timeout if the forwarder does not answer, or ValueError on a
    malformed reply.
    """
    req = dict(kwargs)
    req["cmd"] = cmd
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.settimeout(timeout)
        sock.sendto(json.dumps(req).encode("utf-8"), (host, port))
        data, _ = sock.recvfrom(4096)
    finally:
        sock.close()
    reply = json.loads(data.decode("utf-8"))
    if not isinstance(reply, dict):
        raise ValueError("control reply is not a JSON object")
    return reply
