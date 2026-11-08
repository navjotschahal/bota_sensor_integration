# bota_ft_sensor

ROS 1 (Noetic) integration of a Bota Systems 6-axis force-torque sensor
(SensONE / Rokubi / MedusONE, Gen 0 or Gen A) using Bota's **current official
driver** (`bota-driver`, Bota FT Stack Gen A).

## Why a container

The official driver is a closed pre-built library (`libBotaDriver.so`, PyPI
wheels `manylinux_2_34`, Python >= 3.10). It needs glibc 2.34 and OpenSSL 3, which
Ubuntu 20.04 / Noetic does not have. So the driver runs in a small Ubuntu 24.04
container with host networking, and forwards each frame as one 76-byte UDP
datagram on `127.0.0.1:50100`. Everything ROS-side is plain Python 3.8 and only
parses that stream, so the same stream can feed ROS 2, C++ control loops or
anything else. The wire format is defined once in
`src/bota_ft_sensor/stream_protocol.py` and shared by both sides.

```
sensor --(Bota Socket / EtherCAT / USB)--> [container: bota-driver + bota_forwarder.py]
          --UDP 50100 (frames)-->  bota_stream_node.py  -->  /bota/wrench, /bota/imu, /bota/temperature
          <-UDP 50101 (control)--  /bota/tare (std_srvs/Trigger)
```

## Layout

| path | role |
|---|---|
| `docker/Dockerfile`, `docker/docker-compose.yml` | image with latest `bota-driver`; host network, NET_RAW for EtherCAT |
| `docker/bota_forwarder.py` | runs in the container; lifecycle (configure, tare, activate), blocking reads, UDP out, control port |
| `docker/config/*.json` | driver configs per interface (USB/Bota Binary = default, Bota Socket, EtherCAT Gen A / Gen 0) |
| `src/bota_ft_sensor/stream_protocol.py` | datagram layout, pack/unpack, control requests |
| `scripts/bota_stream_node.py` | ROS node: publishes wrench/imu/temperature, `~tare` service, `/diagnostics` |
| `scripts/bota_container.sh` | `build / start <config> / stop / logs / status / shell` |
| `scripts/bota_stream_probe.py` | ROS-free check: `--status`, `--tare`, `--listen N` |
| `scripts/bota_tare.py` | calls `/bota/tare` |
| `launch/bota_ft_sensor.launch` | starts the node (optionally the container too) |

## One-time setup

1. Docker access for your user (then log out and back in):
   ```bash
   sudo usermod -aG docker $USER
   ```
2. Build the image (pulls the newest `bota-driver` from PyPI):
   ```bash
   rosrun bota_ft_sensor bota_container.sh build
   ```
3. Build the workspace: `catkin_make` in `catkin_ws`.

## Sensor-side setup

**This lab's sensor** (verified 2026-11-08): SensONE Industrial Gen A,
model `BFT-SENS-IND2-B`, serial `SN001597`, firmware V1.0.20. The Bota adapter
box's USB-C port enumerates on the PC as a CDC-ACM virtual COM port:

```
/dev/serial/by-id/usb-Bota_Systems_Bota_Systems_Force_Torque_Sensor_3474375F3335-if00 -> /dev/ttyACM0
```

So the driver speaks **Bota Binary over USB** (`docker/config/bota_binary.json`,
default everywhere). It streams at 1010 Hz with the shipped `app_mode 1 /
app_submode 11`; the driver tares in software on start. `transform_imu_to_wrench_frame`
must be `false`: the driver has no IMU offset table for this product. The
Realtek USB Ethernet adapter on this PC is the lab LAN link, not the sensor.
If the port is not `ttyACM0`, set `BOTA_SERIAL_DEV=/dev/ttyACMx` before `start`.

Other variants, for reference:

* **Ethernet / Bota Socket**: configure *Ethernet - Bota Socket* and a static IP in
  Bota's config tool (https://shop.botasys.com/config-tool), put the host NIC on
  the same subnet, set `sensor_ip_address` in `bota_socket.json`, `start bota_socket.json`.
* **EtherCAT**: direct cable (no switch) to the NIC, `ethercat.json` (Gen A) or
  `ethercat_gen0.json` (Gen 0), `network_interface` = NIC name. The container has
  `NET_RAW`. Use `BOTA_READ_MODE=poll` if the blocking read stalls.

Operation parameters (`app_mode`, `app_submode`, offsets) are authoritative from
the config tool's *Download Config*; drop that JSON into `docker/config/`.

## Run

```bash
rosrun bota_ft_sensor bota_container.sh start                    # USB sensor (bota_binary.json)
rosrun bota_ft_sensor bota_container.sh logs                     # expect "ACTIVE, streaming ..."
roslaunch bota_ft_sensor bota_ft_sensor.launch frame_id:=panda_ft_frame
rostopic hz /bota/wrench
rosrun bota_ft_sensor bota_tare.py                               # hardware tare
rosrun rqt_runtime_monitor rqt_runtime_monitor                    # rate, drops, flags
```

Without ROS: `rosrun bota_ft_sensor bota_stream_probe.py --listen 3`.

## Node interface

Topics (node name `bota` by default): `~wrench` (`geometry_msgs/WrenchStamped`),
`~imu` (`sensor_msgs/Imu`, no orientation), `~temperature`
(`sensor_msgs/Temperature`, every 100th frame), `/diagnostics`.
Service `~tare` (`std_srvs/Trigger`): deactivates the driver, tares, re-activates
(a few frames are skipped). Parameters: `frame_id`, `data_port`, `control_port`,
`control_host`, `stamp_source` (`receive` or `sensor`), `publish_invalid`, `timeout`.

Status bits from the sensor (throttled, overrange, invalid, raw) are forwarded
unchanged. Frames with `invalid` set are dropped unless `publish_invalid` is true.

## Multiple sensors

One container per sensor with distinct `BOTA_DATA_PORT` / `BOTA_CONTROL_PORT`
(set in the environment before `bota_container.sh start`, and a second compose
project name), and one node per port with its own `node_name`.

## Attribution

The driver inside the container is Bota Systems AG's `bota-driver`
(https://botasys.com, https://code.botasys.com), used under its BSD-3-Clause
with attribution licence. Nothing of it is redistributed in this repository.
