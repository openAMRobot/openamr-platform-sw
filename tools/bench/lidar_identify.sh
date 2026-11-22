#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
#
# lidar_identify.sh: identify the navigation LiDAR and why /scan has (or lacks) a publisher.
# Read-only. Commands no robot motion, publishes nothing, calls no service.
#
# Default run (no device access beyond reading): serial devices by-id, udev properties,
# permissions and groups, who holds the port, the running LiDAR node (/rplidar of v0.0.1 or
# /sllidar_node of main), its parameters, `ros2 topic info /scan -v`, `ros2 node info`,
# a bounded `ros2 topic hz /scan` and the TF lookup base_link -> lidar_link.
#
# --probe --port PATH: only when NO LiDAR driver is running and nothing holds the port.
# Starts the pinned sllidar_ros2 node alone (not drivers.launch.py, which also starts the
# micro-ROS agent of the base) for a bounded time at each documented baud rate, 115200
# (RPLIDAR A1) and 1000000 (RPLIDAR S3), and records the S/N, firmware, hardware revision
# and health lines the driver logs. The driver spins the LiDAR's own scan motor while it
# runs; nothing else moves. Its scan goes to /bench/lidar_probe/scan, not /scan.
#
# Usage: lidar_identify.sh [--probe --port PATH] [--seconds N] [--out DIR]
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=tools/bench/bench_common.sh
. "${here}/bench_common.sh"

# Frame names from the platform description (urdf/robo_urdf.urdf.xacro, sensor_frames.yaml)
# and openamrobot_drivers/launch/drivers.launch.py (frame_id lidar_link).
BASE_FRAME=base_link
LIDAR_FRAME=lidar_link
# Documented baud rates: drivers.launch.py LIDAR_MODELS (a1 115200, s3 1000000).
PROBE_BAUDS=(115200 1000000)
LIDAR_NODES=(/sllidar_node /rplidar /rplidar_node)

usage() {
    echo "usage: $0 [--probe --port PATH] [--seconds N] [--out DIR]" >&2
    exit 2
}

probe=0
port=""
seconds=15
out=bench_out
while [ $# -gt 0 ]; do
    case $1 in
        --probe) probe=1; shift ;;
        --port) port=${2:?missing value}; shift 2 ;;
        --seconds) seconds=${2:?missing value}; shift 2 ;;
        --out) out=${2:?missing value}; shift 2 ;;
        *) usage ;;
    esac
done
case $seconds in
    '' | *[!0-9]*) echo "--seconds must be a whole number" >&2; exit 2 ;;
esac
if [ "$seconds" -lt 3 ] || [ "$seconds" -gt 60 ]; then
    { echo "--seconds must be 3 to 60" >&2; exit 2; }
fi
if [ "$probe" -eq 1 ] && [ -z "$port" ]; then
    echo "--probe needs --port (a /dev/serial/by-id path)" >&2
    exit 2
fi

bench_init lidar_identify "$out"

section "1 ROS graph context (run this in the same environment as the launching shell)"
say "ROS_DISTRO=${ROS_DISTRO:-unset} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset (0)} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
say "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-unset} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"
lidar_node=""
if have ros2; then
    step nodes.txt 15 ros2 node list
    for n in "${LIDAR_NODES[@]}"; do
        if grep -qx -- "$n" "${BENCH_OUT}/nodes.txt"; then
            lidar_node=$n
            break
        fi
    done
    say "LiDAR node: ${lidar_node:-none of ${LIDAR_NODES[*]} found}"
else
    say "BLOCKED: ros2 not on PATH (source /opt/ros/jazzy/setup.bash and the workspace)"
    not_checked "every ROS check: ros2 not on PATH"
fi
step processes.txt 5 sh -c "ps -eo pid,etimes,args | grep -E 'rplidar|sllidar' | grep -v grep || true"

section "2 Serial devices, permissions and udev"
step serial_by_id.txt 5 sh -c 'ls -l /dev/serial/by-id/ 2>&1; ls -l /dev/ttyUSB* /dev/ttyACM* 2>&1 || true'
say "user $(id -un), groups: $(id -nG)"
if id -nG | tr ' ' '\n' | grep -qx dialout; then
    say "user is in dialout"
else
    say "user is NOT in dialout: the port needs a permissive udev MODE or the group"
fi
have lsusb && step lsusb.txt 5 lsusb
have lsusb || not_checked "lsusb: not installed (usbutils)"
step udev_rules.txt 5 ls -l /etc/udev/rules.d/
if have systemctl; then
    step modemmanager.txt 5 sh -c 'systemctl is-active ModemManager || true'
fi
devs=()
for d in /dev/serial/by-id/* /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$d" ] && devs+=("$d")
done
if [ "${#devs[@]}" -eq 0 ]; then
    say "no serial device found; check the USB cable and adapter"
fi
i=0
for d in "${devs[@]}"; do
    i=$((i + 1))
    real=$(readlink -f "$d")
    say "device ${d} -> ${real}: $(stat -c '%A %U:%G' "$real")"
    have udevadm && step "udev_${i}.txt" 5 sh -c \
        "udevadm info -q property -n '$real' | grep -E 'ID_SERIAL|ID_VENDOR_ID|ID_MODEL_ID|ID_PATH=|DEVLINKS' || true"
    have fuser && step "holders_${i}.txt" 5 sh -c "fuser -v '$real' 2>&1 || echo 'no process holds ${real}'"
done
have fuser || not_checked "which process holds the port: fuser not installed (psmisc)"
step kernel_log.txt 5 sh -c "journalctl -k -n 400 --no-pager 2>&1 | grep -iE 'cp210|ch34|ttyUSB|ttyACM|usb disconnect' | tail -40 || dmesg 2>&1 | grep -iE 'cp210|ttyUSB' | tail -40 || true"

section "3 Running driver, /scan and TF"
if have ros2; then
    step scan_topic_info.txt 15 ros2 topic info /scan -v
    if [ -n "$lidar_node" ]; then
        step lidar_node_info.txt 15 ros2 node info "$lidar_node"
        for p in serial_port serial_baudrate frame_id scan_mode channel_type angle_compensate; do
            step "param_${p}.txt" 10 ros2 param get "$lidar_node" "$p"
        done
        fid=$(sed -n 's/^String value is: //p' "${BENCH_OUT}/param_frame_id.txt" || true)
        if [ -n "$fid" ] && [ "$fid" != "$LIDAR_FRAME" ]; then
            say "MISMATCH: driver frame_id '${fid}', description frame '${LIDAR_FRAME}'"
        fi
    else
        not_checked "node info and parameters of the LiDAR driver: no driver node in the graph"
    fi
    step scan_hz.txt 12 ros2 topic hz /scan
    step scan_header.txt 10 ros2 topic echo --once --field header /scan
    step tf_base_to_lidar.txt 8 ros2 run tf2_ros tf2_echo "$BASE_FRAME" "$LIDAR_FRAME"
fi
step ros_log.txt 5 sh -c "grep -ihE 'cannot bind|time ?out|health|start scan|scan mode|S/N|Firmware Ver|Hardware Rev' ~/.ros/log/latest/launch.log 2>&1 | tail -40 || true"

section "4 Probe at the documented baud rates"
if [ "$probe" -eq 0 ]; then
    say "skipped: run with --probe --port PATH to read device info at ${PROBE_BAUDS[*]} baud"
    not_checked "device S/N, firmware and hardware revision at each baud rate (--probe not given)"
elif ! have ros2; then
    say "BLOCKED: ros2 not on PATH"
    not_checked "probe: ros2 not on PATH"
elif [ -n "$lidar_node" ]; then
    say "REFUSED: ${lidar_node} is running; stop the bringup first, the probe needs the port alone"
    not_checked "probe: a LiDAR driver was running"
elif [ ! -e "$port" ]; then
    say "BLOCKED: ${port} does not exist"
    not_checked "probe: port missing"
elif have fuser && fuser -s "$(readlink -f "$port")" 2>/dev/null; then
    say "REFUSED: a process holds ${port}; see holders_*.txt"
    not_checked "probe: port held by another process"
else
    for b in "${PROBE_BAUDS[@]}"; do
        # SIGINT, not SIGTERM: a hard kill mid-scan can leave the unit in 80008000
        # (docs/troubleshooting/diagnostics.md, LiDAR issues).
        step "probe_${b}.txt" "$seconds" ros2 run sllidar_ros2 sllidar_node --ros-args \
            -r __node:=bench_lidar_probe -r scan:=/bench/lidar_probe/scan \
            -p channel_type:=serial -p serial_port:="$port" -p serial_baudrate:="$b" \
            -p frame_id:="$LIDAR_FRAME"
        if grep -q 'S/N' "${BENCH_OUT}/probe_${b}.txt"; then
            say "RESULT ${b} baud: device answered:"
            grep -E 'S/N|Firmware Ver|Hardware Rev|health|scan mode' "${BENCH_OUT}/probe_${b}.txt" | tee -a "$BENCH_LOG"
        else
            say "RESULT ${b} baud: no device info"
        fi
        sleep 3
    done
fi

not_checked "the model sticker: photograph it; the driver does not print the model name"
not_checked "/scan rate against the datasheet, angle zero and rotation direction (judged by a person)"
not_checked "TF values against the measured mount (x, z, yaw); this script only shows the lookup"
not_checked "which commit of platform-sw is deployed on the robot (record it by hand)"
bench_finish
