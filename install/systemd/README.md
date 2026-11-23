# systemd units for the robot host

| File | Install to | Purpose |
|---|---|---|
| `openamr.target` | `/etc/systemd/system/` | groups the three services |
| `openamr-agent.service` | `/etc/systemd/system/` | standalone micro-ROS agent (off unless the marker file exists) |
| `openamr-bringup.service` | `/etc/systemd/system/` | `openamrobot_bringup` launch profile |
| `openamr-ui.service` | `/etc/systemd/system/` | operator UI backend and rosbridge |
| `openamr.env` | `/etc/openamr/openamr.env` | `ROS_DOMAIN_ID`, RMW, CycloneDDS file, device, map, profile |
| `cyclonedds.xml` | `/etc/openamr/cyclonedds.xml` | Cyclone DDS interface and buffer settings |
| `bin/openamr-run.sh` | `/opt/openamr/bin/openamr-run.sh` | sources ROS, checks the settings, execs the process |

Install on the Jetson:

```bash
sudo useradd --system --create-home --groups dialout,video openamr
sudo install -d /etc/openamr /opt/openamr/bin /data/maps
sudo install -m 0644 openamr.env cyclonedds.xml /etc/openamr/
sudo install -m 0755 bin/openamr-run.sh /opt/openamr/bin/
sudo install -m 0644 openamr*.service openamr.target /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl start openamr.target          # first runs by hand
sudo systemctl enable openamr.target         # only after the first bench runs passed
```

## Rules these units follow

- **No automatic motion.** No unit publishes `/cmd_vel`, an initial pose, a goal or a Resume.
  Starting, restarting or rebooting only brings the stack up. The base stops by itself on
  `/cmd_vel` silence (200 ms) whatever these units do. The hardwired E-stop is not touched.
- **Order.** `time-sync.target` (chrony) before the agent, the agent before bringup, bringup
  before the UI.
- **One agent only.** `openamrobot_drivers` starts the micro-ROS agent itself, so the H-day
  legacy profile needs no agent unit. `openamr-agent.service` is for a profile whose drivers
  start no agent; it runs only if `/etc/openamr/enable-standalone-agent` exists. Never run both.
- **Restart policy.** `Restart=on-failure` with a start limit (5 in 120 s for the agent, 3 in
  300 s for bringup). A unit that keeps failing stays failed and shows in `systemctl --failed`;
  it does not loop forever.
- **Map.** `openamr-run.sh bringup` exits 3 when `OPENAMR_MAP` is empty or missing. The launch
  has no silent default map on the real robot.
- **Environment.** One file, `openamr.env`, for all units. The same values must be exported on
  the laptop that runs `ros2` commands.

## Open items

- TODO(Karthikeyan): `ROS_DOMAIN_ID`, the CycloneDDS interface name and the rosbridge address.
- TODO(Evans): agent device and baud rate for the NUCLEO-H723ZG. The shipped values are the
  release 001 Teensy ones.
- The UI launch lives in openamrobot-ui and reads its rosbridge parameters from its own
  `config.yaml` (no address, no topic allowlist). `OPENAMR_ROSBRIDGE_ADDRESS` is checked by
  the run script but cannot yet be passed to rosbridge. See `install/jetson/README.md`
  (rosbridge section) for the change the UI repository needs.
- `ReadWritePaths=/data/maps` matches the maps volume of PR #114; the path must exist.
