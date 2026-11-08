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
| `scripts/bota_stream_node.py` | ROS node: wrench/imu/temperature topics, RViz force arrow, `~tare` service, `/diagnostics` |
| `scripts/bota_container.sh` | `build / start <config> / stop / logs / status / shell` |
| `scripts/bota_stream_probe.py` | ROS-free check: `--status`, `--tare`, `--listen N` |
| `scripts/bota_tare.py` | calls `/bota/tare` |
| `scripts/bota_apcf_compare.py`, `launch/bota_apcf_compare.launch` | live sensor-vs-APCF tests: RViz markers, matplotlib window, CSVs |
| `launch/bota_ft_sensor.launch` | starts the node (optionally the container and RViz too) |
| `config/bota_force.rviz` | RViz view: force arrow, sensor axes, TF |

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
(`sensor_msgs/Temperature`, every 100th frame), `~force_marker`
(`visualization_msgs/Marker`, RViz arrow of the measured force), `~force_norm`
(`std_msgs/Float64`, |F| for rqt_plot), `/diagnostics`.
Service `~tare` (`std_srvs/Trigger`): deactivates the driver, tares, re-activates
(a few frames are skipped). Parameters: `frame_id`, `data_port`, `control_port`,
`control_host`, `stamp_source` (`receive` or `sensor`), `publish_invalid`, `timeout`.

Status bits from the sensor (throttled, overrange, invalid, raw) are forwarded
unchanged. Frames with `invalid` set are dropped unless `publish_invalid` is true.

### RViz force arrow

```bash
roslaunch bota_ft_sensor bota_ft_sensor.launch rviz:=true
```

The arrow is drawn at the origin of `marker/frame_id` (defaults to `frame_id`),
scaled by `marker/scale` metres per newton, low-passed at `marker/filter_hz` for
a steady display and published at `marker/rate` (30 Hz). `marker_negate:=true`
draws the reaction force (what the robot feels) instead of the measured one;
`marker/show_torque: true` adds a blue torque arrow. The data topics are never
filtered; only the arrow is. The TF from the sensor to the robot is left to the
consumer: once the mocap-tracked probe exists, publish `world -> <frame_id>` and
set the RViz fixed frame to the robot base to see the arrow at the contact point.

## Sensor vs APCF joint-grid estimator, without mocap

`bota_apcf_compare.py` runs the frame-independent tests live and records them:

```bash
roslaunch bota_ft_sensor bota_ft_sensor.launch            # sensor node (no rviz needed)
roslaunch bota_ft_sensor bota_apcf_compare.launch         # markers + live plot + CSVs
# APCF grid observer running; in its RViz add MarkerArray on /bota_compare/markers
```

* **RViz** (`base_frame`, default `iiwa_link_0`): a gauge beside the robot
  (`gauge_xyz`) with the sensor force arrow in sensor axes, red/blue bars for
  |F| sensor vs |f| APCF, a red sphere at APCF's contact point sized by the sensor
  |F|, and a text HUD with ratio, in_contact, link, existence and the last push summary.
* **Live plot** (system matplotlib, TkAgg), 3x2: left column |F| sensor vs APCF
  with in_contact shading, APCF detector statistic vs threshold and existence,
  APCF contact point xyz; right column fx, fy, fz each with Bota (sensor frame)
  and APCF (base frame). Components are in different frames until the sensor is
  oriented to the robot, so compare shapes and timing, not signs.
* **CSV** in `out_dir` (default `~/bota_tests/<timestamp>/`): `samples.csv`
  (one row per APCF message with the time-matched sensor frame), `sensor.csv`
  (raw 1 kHz stream), `pushes.csv` (per push: duration, mean |F| both, ratio,
  detection on/off latency, link, existence, angle-pair error vs previous pushes,
  Procrustes residual).

Pushes are segmented from the sensor alone (`thr_on` 1 N on, `thr_off` 0.5 N
off). The angle-pair test compares the angle between two pushes as seen by the
sensor with the same angle as seen by APCF, which is invariant to the unknown
sensor orientation as long as the grip stays fixed. Also record a rosbag of
`/bota/wrench /apcf/contacts /apcf/in_contact /apcf/status /tf /tf_static` for
offline work.

## Multiple sensors

One container per sensor with distinct `BOTA_DATA_PORT` / `BOTA_CONTROL_PORT`
(set in the environment before `bota_container.sh start`, and a second compose
project name), and one node per port with its own `node_name`.

## Attribution

The driver inside the container is Bota Systems AG's `bota-driver`
(https://botasys.com, https://code.botasys.com), used under its BSD-3-Clause
with attribution licence. Nothing of it is redistributed in this repository.
