# Operator UI (brief)

The operator UI lives in a **separate repository** — `openamrobot-ui` — so this is a short
pointer, not its documentation. It covers only what a real-robot operator needs to know: the
DDS gotcha that leaves it "connected but empty", and the deployment rule that keeps it from
fighting navigation on Wi-Fi.

Authoritative: the `openamrobot-ui` repo (`docs/REAL-ROBOT-INTEGRATION.md`); memory
`amr-ui-operator`, `amr-wifi-guest-flaky`.

---

## What it is

`openamrobot-ui` — a **React** (CRA + Redux + Blockly) front-end talking **roslib /
rosbridge**, served by **Flask** (:5050), packaged with Docker. The ROS 2 backend
(`openamr_ui_package`) provides rosbridge (:9090), `rosapi`, `web_video_server` (:8080 MJPEG),
and relay nodes that adapt topics for the browser (e.g. `/map` → `/ui/map`, and a
TRANSIENT_LOCAL → VOLATILE QoS conversion the browser needs).

The interface is **already aligned** with the real robot's topics/services — `/goal_pose`,
`/dock_trigger`, `/undock_robot`, `/dock_trigger_status`, `/cmd_vel`, `/scan_filtered`,
`/camera/image_raw`, `/map`, `/amcl_pose`, nav status/cancel — nothing to remap.

---

## Gotcha 1 — DDS: FastDDS default → empty panels

`docker-compose.yml` defaults to **`rmw_fastrtps_cpp`**, but the robot is **CycloneDDS /
domain 0**. FastDDS ≠ CycloneDDS → the UI reports "connected" but every panel is **empty**
("Map not received", "Localization pose missing"). Same class as the PC/`ros2` trap in
[`02_networking_and_dds.md`](02_networking_and_dds.md).

**Fix — the RMW/domain prefix is mandatory** (the shell's FastDDS/42 otherwise overrides the
`.env`):

```bash
cd ~/Documents/openAMRobot/openamrobot-ui
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 docker compose up   # -> http://localhost:5050/control
```

---

## Gotcha 2 — run it ON THE PI, not on the PC

Running the UI Docker **on the PC** over Wi-Fi Guest **saturates the link and blocks
navigation**: its rosbridge + relays + `web_video_server` subscribe *from the PC* to big
RELIABLE topics (`/global_costmap/costmap`, `/map`, `/tf`, `/scan`) → DDS retransmit storm →
`/goal_pose` no longer reaches the Pi. `docker compose down` restores nav immediately. Full
mechanism: [`02_networking_and_dds.md`](02_networking_and_dds.md) Trap 4b.

**Design intent (the correct deployment):** the UI runs **on the robot** (Flask + rosbridge
local to the Pi → all DDS stays on the Pi; only the **browser** crosses Wi-Fi, over one
lightweight websocket). Running it on the PC is a "quick demo" convenience that conflicts with
nav on this network. As of 2026-07-06 the Pi has **no UI installed** (no repo/Docker/colcon
build there) → UI-on-Pi is a from-scratch deploy (later, or over Ethernet). Interim
alternative: put the **PC on wired Ethernet**.

---

## Operator notes

- After a DHCP move, update the UI's rosbridge IP: `openamrobot-ui/.env`
  `REACT_APP_ROSBRIDGE_IP` (and `web/src/shared/constants/index.js`) — prefer `botshare.local`.
- Named locations (e.g. "Station 4") in `blockDefinitions.js` / `flask_app.py` may carry
  **placeholder coordinates** — recalibrate them against the real map before a real drive.
- The AprilTag on-demand gate does **not** touch `/camera/image_raw`, so the UI camera panel
  works regardless of docking state — but that stream is exactly the one that floods Wi-Fi
  Guest (see doc 02); keep it off on the fragile network.
- Fork/branch used for real-robot integration: `SHuttooo:feat/real-robot-integration`.
