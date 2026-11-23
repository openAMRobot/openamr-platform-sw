#!/usr/bin/env bash
# Shared launcher for the openamr-*.service units.
# Usage: openamr-run.sh <agent|bringup|ui>
# Sources ROS and the workspace, checks the settings, then execs the process. It never
# publishes a command: no zero Twist, no goal, no initial pose, no Resume.
set -eu

role="${1:?usage: openamr-run.sh agent|bringup|ui}"

# setup.bash scripts read unset variables.
set +u
# shellcheck disable=SC1090
. "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [ -f "${OPENAMR_WS:-/nonexistent}/setup.bash" ]; then
    # shellcheck disable=SC1091
    . "${OPENAMR_WS}/setup.bash"
fi
set -u

case "$role" in
agent)
    device="${OPENAMR_AGENT_DEVICE:?OPENAMR_AGENT_DEVICE is not set}"
    if [ ! -e "$device" ]; then
        echo "openamr agent: $device does not exist; is the base controller plugged in?" >&2
        exit 3
    fi
    exec ros2 run micro_ros_agent micro_ros_agent serial \
        -b "${OPENAMR_AGENT_BAUD:-115200}" -D "$device"
    ;;
bringup)
    map="${OPENAMR_MAP:-}"
    if [ -z "$map" ] || [ ! -f "$map" ]; then
        echo "openamr bringup: OPENAMR_MAP must name an existing map yaml (got '$map')" >&2
        exit 3
    fi
    launch="${OPENAMR_BRINGUP_LAUNCH:-hday_legacy.launch.py}"
    # shellcheck disable=SC2086
    exec ros2 launch openamrobot_bringup "$launch" map:="$map" ${OPENAMR_BRINGUP_EXTRA_ARGS:-}
    ;;
ui)
    address="${OPENAMR_ROSBRIDGE_ADDRESS:?OPENAMR_ROSBRIDGE_ADDRESS is not set}"
    if [ "$address" = "0.0.0.0" ] || [ -z "$address" ]; then
        echo "openamr ui: refusing to bind rosbridge to all interfaces" >&2
        exit 3
    fi
    # shellcheck disable=SC2086
    exec ros2 launch openamr_ui_bringup ui.launch.py use_sim_time:=false \
        ${OPENAMR_UI_EXTRA_ARGS:-}
    ;;
*)
    echo "openamr-run.sh: unknown role '$role'" >&2
    exit 2
    ;;
esac
