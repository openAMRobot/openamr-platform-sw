# openamrobot_perception

Perception modules for OpenAMRobot. Currently provides the **real-robot LiDAR body
filter**; object detection, point-cloud, and fiducial perception may be added here later.

## `scan_body_filter` node

Removes the robot's own chassis reflections from the LiDAR so they are not treated as
obstacles, then republishes `/scan` as `/scan_filtered`.

This is the **real-robot** filter. It differs from the **simulation** filter
(`openamrobot_nav2/config/scan_body_filter.yaml`, a `laser_filters` angular chain) in two
ways that matter on hardware: it masks **by distance** (keeps real walls behind thin side
posts) and it publishes with a QoS **compatible with our Nav2 costmap observation source** so the
scan is actually delivered. The two are a **sim / real profile pair**, not duplicates.

> **QoS note (corrected):** Nav2 Jazzy does **not** universally require RELIABLE scans — costmap
> SensorData sources default to BEST_EFFORT. What matters is **endpoint compatibility**: a
> BEST_EFFORT publisher with a RELIABLE subscriber is silently dropped. Here our costmap source is
> configured RELIABLE, so we publish RELIABLE to match. Always verify with
> `ros2 topic info <topic> --verbose`.

```bash
ros2 launch openamrobot_perception scan_body_filter.launch.py
# or, with a different calibration:
ros2 launch openamrobot_perception scan_body_filter.launch.py params_file:=/path/to.yaml
```

### Parameters

Defaults are calibrated for **this unit's** LiDAR mount (RPLIDAR A1 mounted rotated
180 deg: 0 deg = robot rear, +/-180 deg = front, -90 = left, +90 = right). Re-measure
(watch `/scan` in RViz) if the mount, chassis, or URDF changes. Values live in
[`config/scan_body_filter_real.yaml`](config/scan_body_filter_real.yaml).

| Parameter | Type | Default | Unit | Meaning / impact |
|---|---|---|---|---|
| `scan_in` | string | `/scan` | topic | input LaserScan |
| `scan_out` | string | `/scan_filtered` | topic | filtered output |
| `reliable_qos` | bool | `true` | — | RELIABLE to **match our RELIABLE-configured costmap source**; QoS must be endpoint-compatible (verify with `ros2 topic info --verbose`) |
| `close_max` | double | `0.40` | m | in "close" sectors, only returns nearer than this are removed |
| `full_mask_sectors_deg` | double[] | `[-45, 49]` | deg | sectors removed at ALL distances (flat `lo,hi,…` pairs) |
| `close_mask_sectors_deg` | double[] | `[-96,-73, 73,96]` | deg | sectors where only `< close_max` returns are removed |

**Failure modes:** angles too wide -> real walls near the body get blanked (robot blind to
close obstacles); `reliable_qos: false` -> BEST_EFFORT, incompatible with our RELIABLE costmap
subscriber -> scan not delivered (match the endpoints, not a universal Nav2 rule); wrong frame convention (mount
not rotated 180 deg) -> masks the wrong side.

### Topics / TF

- Subscribes: `scan_in` (`sensor_msgs/LaserScan`, SensorData QoS).
- Publishes: `scan_out` (`sensor_msgs/LaserScan`, RELIABLE by default).
- No TF is published; the output keeps the input `header.frame_id` (the LiDAR frame).
