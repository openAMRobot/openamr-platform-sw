# Bring-up scripts

## `log_splitter.py` — per-subsystem log topics

The bring-up console is a firehose: every node's logs interleaved. This splits
them so you can watch **one subsystem** at a time.

### What it does

Every ROS 2 node already publishes its logs to **`/rosout`**. `log_splitter`
subscribes to `/rosout` and re-publishes each line as a `std_msgs/String` on a
**per-node** topic under `/logs/`:

```
/rosout ──► log_splitter ──► /logs/dock_trigger
                             /logs/camera
                             /logs/controller_server
                             /logs/apriltag_apriltag
                             /logs/all           (everything, prefixed with the node)
```

Each line carries its level: `[INFO] …`, `[WARN] …`, `[ERROR] …`.
The node name is sanitised into the topic suffix (dots in sub-logger names →
`_`; namespaces keep their `/`).

### Run it (on the Pi)

It subscribes to `/rosout` **locally**, so run it on the Pi — no Wi-Fi cost. You
then echo only the one topic you care about (from the Pi or the PC):

```bash
# on the Pi — dedicated terminal
source /opt/ros/jazzy/setup.bash
source ~/openamr-platform-sw/ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
python3 ~/openamr-platform-sw/ros2/src/openamrobot_bringup/scripts/log_splitter.py
```

Detach it so it survives the SSH session (recipe in memory `amr-pi-ros-commands`):

```bash
setsid bash -c 'source /opt/ros/jazzy/setup.bash; \
  source ~/openamr-platform-sw/ros2/install/setup.bash; \
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0; \
  python3 ~/openamr-platform-sw/ros2/src/openamrobot_bringup/scripts/log_splitter.py \
  > ~/log_splitter.log 2>&1' &
```

### Watch a subsystem (from the PC)

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

ros2 topic echo /logs/dock_trigger --field data      # docking only
ros2 topic echo /logs/camera       --field data      # camera only
ros2 topic echo /logs/apriltag_apriltag --field data # tag detector
ros2 topic echo /logs/all          --field data      # everything, node-prefixed
```

List what's available (one topic appears per node that has logged):

```bash
ros2 topic list | grep '^/logs/'
```

### Notes

- **Discovery, not history**: a `/logs/<node>` topic only appears once that node
  has logged at least once since `log_splitter` started. Start it early.
- **Volatile QoS**: `echo` shows lines from *now* on, not the backlog. For the
  backlog, the bring-up console / `~/log_splitter.log` still have everything.
- `log_splitter` skips its own logs (no feedback loop).
- Not a `ros2 run` entry point — run it with `python3` as above.
