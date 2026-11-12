# Goal routing — how a goal reaches Nav2

Sending a goal on this robot is **not** a direct topic connection: RViz publishes to
`/goal_pose`, but `bt_navigator` listens on `/goal_pose_nav`. Exactly one **forwarder** bridges
the two, and *which* forwarder depends on whether docking is running. Getting this wrong is the
most common "I sent a goal and nothing happened".

---

## 1. The chain

```
RViz "2D Goal Pose"  ──►  /goal_pose  ──►  [ exactly one forwarder ]  ──►  /goal_pose_nav  ──►  bt_navigator
```

Why the indirection: `navigation_launch.py` **remaps** `bt_navigator`'s goal input from
`goal_pose` to **`goal_pose_nav`**:

```python
# navigation_launch.py, bt_navigator node
remappings = remappings + [('goal_pose', 'goal_pose_nav')]
```

This is deliberate — it leaves `/goal_pose` free so the **docking** node can *gate* goals
(undock-before-navigate: if the robot is docked, undock first, then forward the goal). The cost
of that design is that **something must forward `/goal_pose` → `/goal_pose_nav`**, or goals are
silently dropped.

> Use **"2D Goal Pose"** in RViz (publishes `/goal_pose`), **not** the "Nav2 Goal" tool. The
> `nav2_rviz_plugins` GoalTool needs the Navigation2 panel to forward its goal; without that
> panel it publishes nothing usable.

---

## 2. Exactly one forwarder — never two

There must be **exactly one** publisher on `/goal_pose_nav`. Two forwarders = double goals
(Nav2 receives each goal twice and the second preempts the first). Which one runs is chosen by
the `use_docking` argument of `bringup.launch.py`, symmetrically for sim and real:

| Mode | Forwarder | Launched by |
|---|---|---|
| **Docking on** (`use_docking:=true`, default) | `dock_trigger` **owns** `/goal_pose` and republishes to `/goal_pose_nav` (after undocking if docked) — it *is* the forwarder | `openamrobot_docking` launch (`openamrobot_docking.launch.py` sim / `docking_real.launch.py` real) |
| **Nav-only** (`use_docking:=false`) | a plain `topic_tools relay /goal_pose → /goal_pose_nav` | `goal_relay.launch.py` |

Both branches are gated in `bringup.launch.py` so **exactly one** runs. If you compose layers by
hand (per-terminal workflow), you must pick one yourself:

```bash
# Nav-only: run the relay
ros2 launch openamrobot_bringup goal_relay.launch.py

# Docking: do NOT run the relay — the docking layer's dock_trigger is the forwarder
```

`goal_relay.launch.py` is a one-node launch:

```python
Node(package='topic_tools', executable='relay', name='goal_pose_relay',
     arguments=['/goal_pose', '/goal_pose_nav'])
```

As soon as the docking layer is up, it owns the routing — do not also start the relay.

---

## 3. Diagnosing goal routing

```bash
# Is a forwarder present, and only one?
ros2 topic info /goal_pose_nav --verbose      # publisher count == 1

# Is bt_navigator actually listening there (not on /goal_pose)?
ros2 node info /bt_navigator | grep goal_pose_nav

# Does a goal arrive when you click "2D Goal Pose"?
ros2 topic echo /goal_pose_nav                # should print your PoseStamped
```

Symptoms and causes:

| Symptom | Cause |
|---|---|
| Goal published, robot does nothing | No forwarder running (nav-only without `goal_relay`, or docking layer down) |
| Goal executes twice / preempts itself | **Two** forwarders on `/goal_pose_nav` (relay *and* dock_trigger both up) |
| "Nav2 Goal" tool does nothing | Wrong RViz tool — use **2D Goal Pose** |
| Goal accepted but robot won't move | Not a routing problem — see costmaps ([`02_costmaps.md`](02_costmaps.md)), sub-stiction yaw / teleop ([`04_real_robot_tuning.md`](04_real_robot_tuning.md)), or power ([`../safety/02_limits_and_watchdog.md`](../safety/02_limits_and_watchdog.md)) |

---

## 4. The action interface underneath

`bt_navigator` exposes the standard Nav2 actions (`navigate_to_pose`,
`navigate_through_poses`); the `/goal_pose_nav` topic is a convenience entry that the BT turns
into a `NavigateToPose` goal. Docking Phase 1 uses the **action** directly (`NavigateToPose` to a
staging pose) rather than the topic — see
[`../../ros2/src/openamrobot_docking/docs/02_architecture.md`](../../ros2/src/openamrobot_docking/docs/02_architecture.md).

---

## Cross-links

- Where `bt_navigator` sits in the graph → [`01_architecture.md`](01_architecture.md)
- Docking's undock-before-navigate gate → [`../../ros2/src/openamrobot_docking/docs/`](../../ros2/src/openamrobot_docking/docs/README.md)
- Troubleshooting matrix → [`06_troubleshooting.md`](06_troubleshooting.md)
