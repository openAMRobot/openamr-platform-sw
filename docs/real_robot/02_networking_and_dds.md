# Networking & DDS

The real robot lives on Wi-Fi, and the transport is the single most common source of
"the robot is broken" scares that turn out **not** to be the robot. This doc records the
DDS configuration, the traps, and the Wi-Fi failure modes — with the reflex that fixes
most of them: **check the network before blaming the software.**

Consolidates memory `amr-wifi-guest-flaky`, `amr-pi-ros-commands`, `pi-ssh-access`,
`amr-ui-operator`, and the 2026-07-06 day log.

---

## The configuration

| Setting | Value | Why |
|---|---|---|
| `RMW_IMPLEMENTATION` | **`rmw_cyclonedds_cpp`** | Whole stack moved to CycloneDDS on 2026-06-18 (required by the Nav2 / docking actions; FastDDS has a Jazzy Python crash that silently breaks `dock_trigger.py` action goals). |
| `ROS_DOMAIN_ID` | **0** | The Pi sets nothing → domain 0. The dev PC defaults to **42**. |
| Discovery | multicast | DDS discovery is multicast → it does **not** cross a router. PC and Pi must be on the **same subnet** (both `172.17.x.x/16`). |
| Reach | `botshare.local` (mDNS) | DHCP IP changes; the mDNS name follows the SSD. |

Every terminal — PC and Pi — must export the first two or it sees nothing:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
```

---

## Trap 1 — the PC's FastDDS / domain 42 default

A fresh PC shell is **FastDDS on domain 42**. It will "connect" to nothing on the robot and
`ros2 topic list` comes back empty. This is not a robot fault — you forgot the export block.
After changing the domain, bounce the daemon:

```bash
ros2 daemon stop && ros2 daemon start
ros2 topic list        # should now show /scan, /odom, /map, /camera/... from the Pi
```

The same trap bites the **operator UI** (Trap 4) and any Docker container that inherits the
shell env. The simplest way to avoid it entirely: **work directly on the Pi over SSH**,
where the env is already correct.

---

## Trap 2 — the DHCP IP changes; use mDNS

The Pi's address is DHCP and **moves**. On 2026-07-06 a hardware swap (same SSD, new MAC)
took it from `172.17.201.29` (now dead) to `172.17.17.64`. **Never hard-code the IP.** Use
`botshare.local` (mDNS), which follows the SSD:

```bash
ssh botshare@botshare.local
getent hosts botshare.local          # resolve the current IP when you actually need it
```

Anything that stores an IP must be updated after a move — notably the operator UI's
rosbridge IP (`openamrobot-ui/.env` `REACT_APP_ROSBRIDGE_IP`, and
`web/src/shared/constants/index.js`).

---

## Trap 3 — Wi-Fi Guest degrades → mDNS + DDS drop (looks like a robot failure)

The robot is on **`Motionlab-Guest`**, an isolated and unstable guest network. When it
degrades, the symptoms **mimic a hardware fault** but are pure network:

- `ros2 topic echo` timeouts from the PC,
- `getent hosts botshare.local` fails (mDNS gone),
- `ping botshare.local` = 100 % loss,
- "the lidar stopped", "the robot won't move".

On 2026-07-06 all of the above happened while the **lidar was fine the whole time** (`/scan`
~6.8 Hz, checked *on the Pi*). It even glitched on a phone — that's the network, not ROS.

### The reflex: network before robot

```bash
# 1. can we even resolve + reach the Pi?
getent hosts botshare.local
ping -c 3 botshare.local

# 2. did the PC roam onto a different SSID? (must be the SAME network as the Pi = Guest)
nmcli -t -f ACTIVE,SSID dev wifi | grep '^yes'

# 3. check the sensor DIRECTLY ON THE PI, not through the flaky Wi-Fi
ssh botshare@botshare.local
#   (on the Pi, env sourced)
ros2 topic hz /scan          # ~5.5–10 Hz on an A1 = healthy
```

If (1) or (2) fail, it's Wi-Fi — stop debugging ROS. Do **not** switch the PC to
`Motionlab-Member` to "reach the robot": you must be on the **same** network as the Pi,
which is Guest.

---

## Trap 4 — big RELIABLE topics saturate the link (retransmit storm)

This is the mechanism behind most link collapses. Two triggers, same root cause.

### 4a. The camera stream

The IMX708 publishes **1280×720 RELIABLE** (~2.76 MB/frame, ~15 fps). On a healthy link
(earlier sessions, ~0 % loss) it's fine. On a **degraded** guest link, any packet loss
triggers a **DDS reliable-retransmit storm** on those big image frames → it eats the airtime
→ ping / SSH / DDS all collapse. **The camera didn't change — the network degraded.**

- Trigger observed 2026-07-06: clicking **"Start Camera"** in the UI (starts
  `web_video_server` pulling the stream), and adding a PC subscriber (RViz Image display, or
  the UI's `web_video_server` on `/camera/image_raw`).
- Verified fix: **light bring-up** `use_camera:=false use_docking:=false` → connection stable
  (0 % loss, ~14 ms), the PC sees all ~46 nav topics again, DDS discovery OK.
- For the "go to Station 4" nav demo you need **no camera and no docking** — the light
  profile is the correct default on this network. If the camera is genuinely required later
  (UI video panel), the fix is the **network** (wired Ethernet, or drop the stream
  resolution/fps), **not** the software.

> Never add a raw `/camera/image_raw` Image display in RViz over Wi-Fi — use
> `rqt_image_view` with the **compressed** transport, or grab a snapshot on the Pi.

### 4b. The UI Docker running ON THE PC

Even with the camera off, launching the operator UI's Docker stack **on the PC** blocked
navigation: RViz goals stopped reaching the Pi (no path drawn, `/cmd_vel_nav` silent). The
container's nodes (rosbridge + relays + `web_video_server`) **subscribe from the PC** to big
RELIABLE topics — `/global_costmap/costmap`, `/map`, `/tf`, `/scan` — → same retransmit storm
→ `/goal_pose` PC→Pi no longer gets through. `docker compose down` → **navigation resumes
immediately** (confirmed).

**Implication:** do **not** run the UI Docker on the PC at the same time as nav on this
Wi-Fi. The right deployment is **UI ON THE PI** (Flask + rosbridge local to the Pi, all DDS
stays on the Pi; only the browser crosses Wi-Fi, over one lightweight websocket). Detail:
[`06_operator_ui.md`](06_operator_ui.md).

---

## Trap 5 — `docker compose` inherits the wrong RMW/domain

`docker compose up` **without** the RMW/domain prefix inherits the shell's FastDDS / domain
42 → the container says "connected" but every panel is **empty** ("Map not received",
"Localization pose missing"). The prefix is **mandatory**:

```bash
cd ~/Documents/openAMRobot/openamrobot-ui
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 docker compose up   # -> http://localhost:5050/control
```

Same class as Trap 1. (And on this Wi-Fi, prefer running it on the Pi anyway — Trap 4b.)

---

## Summary — network failure matrix

| Symptom | Real cause | Check / fix |
|---|---|---|
| PC `ros2 topic list` empty | FastDDS/42 default | export the CycloneDDS/0 block; `ros2 daemon stop && start` |
| `botshare.local` won't resolve / 100 % ping loss | Wi-Fi Guest degraded (or Pi rebooted/brown-out) | `getent hosts` + `ping`; check sensors **on the Pi**; check PC didn't roam SSID |
| Can ping but no robot topics | different subnet / RMW / domain | same subnet, `rmw_cyclonedds_cpp`, domain 0 |
| Link collapses when camera starts | RELIABLE image flood on degraded Wi-Fi | light bring-up (`use_camera:=false`); Ethernet for real camera use |
| Nav goals don't reach the Pi while UI up | UI-on-PC pulling big RELIABLE topics | `docker compose down`; run UI on the Pi |
| UI connected but panels empty | container on FastDDS/42 | launch with the `RMW/ROS_DOMAIN_ID` prefix |
| "lidar/robot broken" but sensors fine on Pi | it's the Wi-Fi | network reflex above |

Power brown-out is a *separate* false culprit that also drops the network — see
[`07_troubleshooting.md`](07_troubleshooting.md) (ping before blaming software).
