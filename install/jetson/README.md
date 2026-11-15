# Jetson install: Orin NX 8 GB on the Seeed reComputer Robotics J401

Target: JetPack 7.2.1 (Jetson Linux 39.2.1, Ubuntu 24.04, kernel 6.8) with ROS 2 Jazzy native
from apt. Docker is not needed for the base stack. A Jazzy container on JetPack 6.2 (Ubuntu
22.04) is the fallback only, if the JetPack 7.2 image fails on the unit.

Status: skeleton. `install-jetson.sh` has not been run on a unit. Every step has a check and the
script reports a step as done only when its check passes.

## Step 0: flash (cannot be scripted here)

On a separate host PC with Ubuntu 22.04 or 20.04 (the flash host must not be 24.04), put the
Orin NX in recovery mode (it shows as USB ID 0955:7423 for the 8 GB module) and flash the Seeed
JetPack 7.2 image with `l4t_initrd_flash.sh --flash-only --massflash 1 --network usb0`, following the
Seeed flash table. The product is "reComputer Robotics" on the "Robotics J401" carrier (J4011 =
Orin NX 8 GB). The BOM nickname "Robotics 104" is not a Seeed product name: record the exact SKU.
TODO(Karthikeyan): SKU, whether the unit already has JetPack 7.2, `cat /etc/nv_tegra_release`.

## Steps 1 to 11 (`install-jetson.sh list`)

| Step | Check |
|---|---|
| identity | `/etc/nv_tegra_release` is R39 revision 2.1, Ubuntu 24.04, aarch64 |
| ros_apt | ROS apt source and key present |
| ros_packages | every package in `APT_ROS_PACKAGES` and `APT_HOST_PACKAGES` is installed |
| microros_agent | `ros2 pkg prefix micro_ros_agent` succeeds (agent built from the `jazzy` branch; record the commit) |
| udev | `/etc/udev/rules.d/99-openamrobot.rules` equals the repository file |
| chrony | `conf.d/openamr.conf` installed, chrony active, `chronyc tracking` answers |
| can_modules | `can`, `can_raw`, `mttcan` loaded |
| sysctl | `ipfrag_time=3`, `rmem_max=2147483647` |
| services | unit files, run script, env file and CycloneDDS file installed (not enabled) |
| workspace | `/opt/openamr/ros2/install/setup.bash` exists |
| evidence | `/var/lib/openamr/install-evidence.txt` written with script hash, release, versions |

```bash
./install-jetson.sh check            # changes nothing
./install-jetson.sh --dry-run run    # prints what would run
./install-jetson.sh run              # stops at the first step that fails to verify
```

Not installed by the script, by design:

- **ZED SDK** 5.5.1 for JetPack 7.2 and `zed-ros2-wrapper` v5.5.0: the SDK installer is large;
  download it to the bench beforehand. The ZED node is the one node that needs the host CUDA.
- **Gemini 336L firmware**: check it is 1.8.10 (or the version in the Orbbec docs) before H-day.
- **`rplidar_ros`**: `sllidar_ros2` is not in the Jazzy rosdistro and its README lists Humble and
  Foxy. The Slamtec `rplidar_ros` (ros2 branch) is released for Jazzy and lists the S3. The
  drivers launch still starts `sllidar_ros2`; moving it is a driver decision, not an install one.
- **OpenArm** (`openarm_ros2`, `libopenarm-can-dev`): owned by the upper-body repository.
  CAN FD bus setup, termination and `restart-ms` belong to that package's scripts.

## Clock policy (chrony)

`chrony/openamr.conf` makes the Jetson the time master of the robot network; `chrony/openamr-tablet.conf`
is the client reference. Reasons and rules are in the file header. Summary:

1. Jetson syncs upstream when it can and serves the robot network; without upstream it serves its
   own clock (`local stratum 10 orphan`), so tablet and robot still agree.
2. The tablet uses the Jetson as its only time source.
3. The MCU is mapped to the Jetson clock by micro-ROS agent time sync, repeated on reconnect.
4. Boot may step the clock (first 3 updates, over 1 s); afterwards only slewing.
5. Target Jetson to tablet offset below 5 ms. Evidence: `chronyc tracking`, `chronyc clients`.

TODO(Karthikeyan): the robot subnet in `allow`, the allowed upstream servers, the Jetson address
for the tablet, and how the Getac tablet is synced (Windows `w32tm` or other).

## rosbridge: bind address and topic allowlist (note for the UI repository)

Verified for rosbridge_suite 2.7.1 on Jazzy: the websocket server binds to all interfaces by
default (`address` empty), has no authentication, and `topics_glob` and `services_glob` are empty,
which exposes every topic and service. With the UI contract that includes raw `/cmd_vel`.
The UI launch (`openamr_ui_package/launch/new_ui_launch.py`) passes only `port: 9090` to rosbridge.

Recommended settings, to be made in the UI repository (not changed here, it is another repository):

- `address`: the robot address on the tablet network, never `0.0.0.0`
  (`OPENAMR_ROSBRIDGE_ADDRESS` in `install/systemd/openamr.env`; the run script refuses `0.0.0.0`).
- `ssl: true` with a certificate. TODO(Karthikeyan): certificate handling.
- `topics_glob` and `services_glob` limited to the UI contract. The tablet may publish only the
  UI command topic that the arbitration chain accepts (platform-sw #90), the goal, initial pose
  and dock topics, and may subscribe to the status, map, scan and pose topics it draws. It must
  not publish to `/cmd_vel`, `/cmd_vel_nav` or controller topics.
- The tablet does not join DDS over Wi-Fi. It talks to rosbridge only.
- If any container uses Fast DDS: `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`.
