# bota_sensor_integration

Backend for a Bota Systems 6-axis force-torque sensor (SensONE / Rokubi /
MedusONE, Gen 0 or Gen A). One responsibility: get frames out of the sensor and
onto a local socket, reliably, with tare and status. Any client (ROS 1, ROS 2,
plain C++ or Python) reads that stream; none of them touch the driver.

```
sensor --(USB | Bota Socket | EtherCAT)--> [container: official bota-driver + bota_forwarder.py]
                                            --UDP 50100 (one 76-byte datagram per frame)--> clients
                                            <-UDP 50101 (JSON control: tare, status, ping)--
```

## Why a container

Bota's current driver is a closed pre-built library (`libBotaDriver.so`, PyPI
`bota-driver` wheels `manylinux_2_34`, Python >= 3.10). It needs glibc 2.34 and
OpenSSL 3, which Ubuntu 20.04 lacks. So the driver runs unchanged in an Ubuntu
24.04 container with host networking and forwards frames over loopback UDP.
On a 22.04+ host the same forwarder runs natively without the container.

## Layout

| path | role |
|---|---|
| `docker/Dockerfile`, `docker/docker-compose.yml` | image with the latest `bota-driver`; host network, NET_RAW (EtherCAT), serial passthrough (USB) |
| `docker/bota_forwarder.py` | runs in the container: lifecycle (configure, tare, activate), blocking reads, UDP out, control port |
| `docker/config/*.json` | driver configs per interface (USB/Bota Binary = default, Bota Socket, EtherCAT Gen A / Gen 0) |
| `src/bota_ft_sensor/stream_protocol.py` | the wire format: datagram layout, pack/unpack, control requests. The contract for all clients. |
| `scripts/bota_container.sh` | `build / start [config] / stop / restart / logs / status / shell` |
| `scripts/bota_stream_probe.py` | ROS-free check: `--status`, `--tare`, `--listen N` |

Clients are kept out of this repo. The ROS 1 nodes (sensor publisher, tare,
live comparison against the APCF contact estimator) live in the `cpf` repo under
`src/ros1/bota_nodes/` and import `bota_ft_sensor.stream_protocol` from here.

## Wire format (protocol version 1)

One UDP datagram per frame, little-endian, 76 bytes, `struct` format
`<4sBBHQIHH6f3f3ff`: magic `BOTA`, protocol version, driver state, 16-bit
sequence (drop detection), forwarder `time_ns`, sensor timestamp [us], 16-bit
status word (bit0 throttled, bit1 overrange, bit2 invalid, bit3 raw), reserved,
force xyz [N], torque xyz [Nm], acceleration xyz [m/s^2], angular rate xyz
[rad/s], temperature [C]. Control: JSON `{"cmd": "tare" | "status" | "ping"}`
to UDP 50101, JSON reply. Full definition in `stream_protocol.py`.

## One-time setup

```bash
sudo usermod -aG docker $USER            # then log out and back in
rosrun bota_ft_sensor bota_container.sh build   # or: docker compose -f docker/docker-compose.yml build
```

The package is a minimal catkin package only so that the python module lands
on the workspace path; nothing in it needs ROS. Without ROS: `cd` here and
`PYTHONPATH=src python3 scripts/bota_stream_probe.py --status`.

## Sensor-side setup

**This lab's sensor** (verified 2026-11-08): SensONE Industrial Gen A, model
`BFT-SENS-IND2-B`, serial `SN001597`, firmware V1.0.20. The Bota adapter box's
USB-C port enumerates as a CDC-ACM virtual COM port:

```
/dev/serial/by-id/usb-Bota_Systems_Bota_Systems_Force_Torque_Sensor_3474375F3335-if00 -> /dev/ttyACM0
```

So the driver speaks **Bota Binary over USB** (`docker/config/bota_binary.json`,
the default). It streams at 1010 Hz with `app_mode 1 / app_submode 11`; the driver
tares in software on start. `transform_imu_to_wrench_frame` must be `false`: the
driver has no IMU offset table for this product. If the port is not `ttyACM0`,
set `BOTA_SERIAL_DEV=/dev/ttyACMx` before `start`.

Other variants: **Bota Socket** (Ethernet): select it and a static IP in Bota's
config tool (https://shop.botasys.com/config-tool), set `sensor_ip_address` in
`bota_socket.json`, `start bota_socket.json`. **EtherCAT**: direct cable to the
NIC, `ethercat.json` / `ethercat_gen0.json` with `network_interface`; the
container has NET_RAW; `BOTA_READ_MODE=poll` if the blocking read stalls.
Operation parameters are authoritative from the config tool's *Download Config*.

## Run

```bash
rosrun bota_ft_sensor bota_container.sh start          # USB sensor; restart policy keeps it up
rosrun bota_ft_sensor bota_container.sh logs           # "ACTIVE, streaming ... 1010 Hz"
rosrun bota_ft_sensor bota_stream_probe.py --listen 3  # frames, rate, lost count
rosrun bota_ft_sensor bota_stream_probe.py --tare
```

Forwarder environment (compose): `BOTA_CONFIG`, `BOTA_DATA_HOST` (127.0.0.1),
`BOTA_DATA_PORT` (50100), `BOTA_CONTROL_PORT` (50101), `BOTA_TARE_ON_START` (1),
`BOTA_READ_MODE` (blocking | poll), `BOTA_SERIAL_DEV`.

Verified on hardware: ~1017 Hz, 0 lost frames, tare OK, ~0.06 ms loopback latency.

## Multiple sensors

One container per sensor with distinct `BOTA_DATA_PORT` / `BOTA_CONTROL_PORT`
and compose project name; each client reads its own port.

## Roadmap

C++ forwarder with a real-time read thread (the Python one is the proven first
step), multicast fan-out for several consumers, optional shared-memory
latest-frame output for same-host 1 kHz control loops, header-only C++ parser.

## Attribution

The driver inside the container is Bota Systems AG's `bota-driver`
(https://botasys.com, https://code.botasys.com), used under its BSD-3-Clause
with attribution licence. Nothing of it is redistributed in this repository.
