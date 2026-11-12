# openamrobot_perception

Perception modules for OpenAMRobot. Currently provides the **LiDAR body filter**, used by
both the real robot and simulation; object detection, point-cloud, and fiducial perception
may be added here later.

## `scan_body_filter` node

Removes the robot's own chassis reflections from the LiDAR so they are not treated as
obstacles, then republishes `/scan` as `/scan_filtered`. Masks **by distance** in some
sectors (keeps real walls behind thin side posts, drops only near-range returns there) and
**always** in others, and publishes with a QoS **compatible with our Nav2 costmap observation
source** so the scan is actually delivered.

**One node, two calibrations — not a sim/real pair of different filters.** Before issue #74 /
PR #97, simulation ran a separate mechanism: a `laser_filters` angular-bounds chain
(`openamrobot_nav2/config/scan_body_filter.yaml`), a pure *truncation* filter — it keeps
exactly one contiguous angular window and resizes the scan to it, so it cannot mask an
interior sector while leaving both wraparound ends clear. That shape fit the body footprint's
old, rear-centered location, but not where it ended up after the 12 Oct 2026 LiDAR mount
rotation (now near the sensor's own 0°). Sim now runs this same node instead, pointed at its
own config
([`openamrobot_nav2/config/scan_body_filter_sim.yaml`](../openamrobot_nav2/config/scan_body_filter_sim.yaml),
which has the full re-measurement and the mask it replaced), alongside this package's own
real-robot config (`config/scan_body_filter_real.yaml`, below).

> **Empty-array parameter gotcha:** to disable a sector list for a profile that needs no
> close-range masking, leave `close_mask_sectors_deg` out of that profile's params YAML
> entirely — do not set it to `[]`. A YAML override of an array parameter to an empty list has
> no element to infer a type from, so `rclpy` loads it as `Type.NOT_SET` rather than an empty
> `DOUBLE_ARRAY`; that silently defeats this node's own `declare_parameter()` default, and the
> next `get_parameter()` call raises `rclpy.exceptions.ParameterUninitializedException` at
> startup. Omitting the key entirely falls back to this node's own Python-side default instead
> (hit and fixed this way in the sim config, issue #74, 26 Oct 2026).

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

Defaults are calibrated for **this unit's** LiDAR mount (RPLIDAR A1, legacy (existing
robot), mounted rotated 180 deg: 0 deg = robot rear, +/-180 deg = front, -90 = left,
+90 = right). Re-measure (watch `/scan` in RViz) if the mount, chassis, or URDF changes,
and on the RPLIDAR S3 (OpenAMRobot 2.0 navigation LiDAR) before relying on these values.
Values live in [`config/scan_body_filter_real.yaml`](config/scan_body_filter_real.yaml).

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
