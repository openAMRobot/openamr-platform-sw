#!/usr/bin/env bash
# OpenAMRobot 2.0 Jetson install: Orin NX 8 GB on the Seeed reComputer Robotics J401 carrier.
# JetPack 7.2.1 (Jetson Linux 39.2.1, Ubuntu 24.04), native ROS 2 Jazzy from apt.
#
# This is a CHECKED SKELETON. Each step has a "do" function and a "check" function; a step
# counts as done only when its check passes. Nothing here has been run on a unit yet.
#
#   install-jetson.sh list              print the steps
#   install-jetson.sh check [step]      run the checks only (changes nothing)
#   install-jetson.sh run [step]        run each step whose check fails, then re-check it
#   install-jetson.sh --dry-run run     print what would run
#
# Step 0 (flashing) happens on a separate Ubuntu 22.04 or 20.04 host PC and cannot be run
# here; see README.md. This script starts on the booted Jetson.
set -u

DRY_RUN=0
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="${OPENAMR_STATE_DIR:-/var/lib/openamr}"
LOG_FILE="${OPENAMR_INSTALL_LOG:-/tmp/openamr-install.log}"

# --- pinned versions. TODO(Karthikeyan): confirm each pin on the unit before H-day. -------
EXPECT_L4T_RELEASE="${EXPECT_L4T_RELEASE:-R39}"          # JetPack 7.2.x is Jetson Linux 39.2.x
EXPECT_L4T_REVISION="${EXPECT_L4T_REVISION:-2.1}"        # 7.2.1 = 39.2.1
EXPECT_UBUNTU="${EXPECT_UBUNTU:-24.04}"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
ZED_SDK_VERSION="${ZED_SDK_VERSION:-5.5.1}"              # SDK release for JetPack 7.2
ZED_WRAPPER_TAG="${ZED_WRAPPER_TAG:-v5.5.0}"
ORBBEC_FW_EXPECTED="${ORBBEC_FW_EXPECTED:-1.8.10}"       # Gemini 336L firmware in the SDK docs
NV_TEGRA_RELEASE_FILE="${NV_TEGRA_RELEASE_FILE:-/etc/nv_tegra_release}"
OS_RELEASE_FILE="${OS_RELEASE_FILE:-/etc/os-release}"

# Packages from apt (names as in the Jazzy rosdistro). Packages that are not in apt for
# Jazzy are listed in README.md and are NOT installed here.
APT_ROS_PACKAGES=(
    ros-jazzy-ros-base ros-jazzy-navigation2 ros-jazzy-nav2-bringup
    ros-jazzy-robot-localization ros-jazzy-twist-mux ros-jazzy-rosbridge-server
    ros-jazzy-rplidar-ros ros-jazzy-orbbec-camera-v2 ros-jazzy-rmw-cyclonedds-cpp
    ros-jazzy-tf2-tools ros-jazzy-xacro ros-jazzy-robot-state-publisher
    ros-jazzy-topic-tools
)
APT_HOST_PACKAGES=(chrony can-utils python3-colcon-common-extensions python3-rosdep
                   python3-pytest git curl)

log() { printf '%s %s\n' "$(date -u +%FT%TZ)" "$*" | tee -a "$LOG_FILE" >&2; }
run() {
    if [ "$DRY_RUN" = 1 ]; then log "DRY-RUN: $*"; return 0; fi
    log "RUN: $*"; "$@"
}

# ---- step 1: identify the unit -----------------------------------------------------------
check_identity() {
    [ -r "$NV_TEGRA_RELEASE_FILE" ] || { log "no $NV_TEGRA_RELEASE_FILE: not a Jetson image"; return 1; }
    head -1 "$NV_TEGRA_RELEASE_FILE" | grep -q "$EXPECT_L4T_RELEASE" \
        || { log "Jetson Linux is not $EXPECT_L4T_RELEASE: $(head -1 "$NV_TEGRA_RELEASE_FILE")"; return 1; }
    head -1 "$NV_TEGRA_RELEASE_FILE" | grep -q "REVISION: $EXPECT_L4T_REVISION" \
        || { log "Jetson Linux revision is not $EXPECT_L4T_REVISION"; return 1; }
    grep -q "^VERSION_ID=\"$EXPECT_UBUNTU\"" "$OS_RELEASE_FILE" \
        || { log "Ubuntu is not $EXPECT_UBUNTU"; return 1; }
    [ "$(uname -m)" = "aarch64" ] || { log "not aarch64"; return 1; }
}
do_identity() {
    log "The image is wrong or the unit is not flashed. Flash JetPack 7.2.1 first (README.md,"
    log "step 0) and record: carrier SKU, module, 'cat /etc/nv_tegra_release'."
    return 1
}

# ---- step 2: apt sources and ROS key ------------------------------------------------------
check_ros_apt() { grep -rqs "packages.ros.org/ros2/ubuntu" /etc/apt/sources.list.d /etc/apt/sources.list \
                  && [ -s /usr/share/keyrings/ros-archive-keyring.gpg ]; }
do_ros_apt() {
    run sudo apt-get update
    run sudo apt-get install -y software-properties-common curl
    run sudo add-apt-repository -y universe
    run sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    if [ "$DRY_RUN" = 0 ]; then
        local codename
        codename="$(sed -n 's/^UBUNTU_CODENAME=//p' "$OS_RELEASE_FILE")"
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $codename main" \
            | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    fi
    run sudo apt-get update
}

# ---- step 3: ROS 2 Jazzy and host packages ------------------------------------------------
check_ros_packages() {
    local p
    for p in "${APT_ROS_PACKAGES[@]}" "${APT_HOST_PACKAGES[@]}"; do
        dpkg -s "$p" >/dev/null 2>&1 || { log "missing package: $p"; return 1; }
    done
    [ -f "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ]
}
do_ros_packages() { run sudo apt-get install -y "${APT_HOST_PACKAGES[@]}" "${APT_ROS_PACKAGES[@]}"; }

# ---- step 4: micro-ROS agent (not in the Jazzy apt index; built from micro_ros_setup) ------
check_microros_agent() {
    # shellcheck disable=SC1090
    ( set +u; . "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
      # shellcheck disable=SC1091
      [ -f /opt/openamr/microros_ws/install/setup.bash ] && . /opt/openamr/microros_ws/install/setup.bash
      ros2 pkg prefix micro_ros_agent >/dev/null 2>&1 )
}
do_microros_agent() {
    # TODO(Karthikeyan): record the micro_ros_setup and agent commit in the evidence pack.
    log "Build the agent from micro_ros_setup branch jazzy into /opt/openamr/microros_ws:"
    log "  ros2 run micro_ros_setup create_agent_ws.sh && build_agent.sh"
    return 1
}

# ---- step 5: udev rules --------------------------------------------------------------------
check_udev() {
    cmp -s "$HERE/../udev/99-openamrobot.rules" /etc/udev/rules.d/99-openamrobot.rules
}
do_udev() {
    run sudo install -m 0644 "$HERE/../udev/99-openamrobot.rules" /etc/udev/rules.d/99-openamrobot.rules
    run sudo udevadm control --reload-rules
    run sudo udevadm trigger
}

# ---- step 6: chrony --------------------------------------------------------------------------
check_chrony() {
    cmp -s "$HERE/chrony/openamr.conf" /etc/chrony/conf.d/openamr.conf \
        && systemctl is-active --quiet chrony \
        && chronyc tracking >/dev/null 2>&1
}
do_chrony() {
    run sudo install -d /etc/chrony/conf.d
    run sudo install -m 0644 "$HERE/chrony/openamr.conf" /etc/chrony/conf.d/openamr.conf
    run sudo systemctl enable --now chrony
    run sudo systemctl restart chrony
}

# ---- step 7: CAN tools and kernel modules (arm buses are configured by the arm package) ------
check_can_modules() { lsmod | grep -q '^mttcan' && lsmod | grep -q '^can_raw'; }
do_can_modules() {
    run sudo modprobe can
    run sudo modprobe can_raw
    run sudo modprobe mttcan
}

# ---- step 8: network tuning for DDS over a wired interface ----------------------------------
check_sysctl() {
    [ "$(sysctl -n net.ipv4.ipfrag_time 2>/dev/null)" = "3" ] \
        && [ "$(sysctl -n net.core.rmem_max 2>/dev/null)" -ge 2147483647 ]
}
do_sysctl() {
    if [ "$DRY_RUN" = 0 ]; then
        printf 'net.ipv4.ipfrag_time=3\nnet.ipv4.ipfrag_high_thresh=134217728\nnet.core.rmem_max=2147483647\n' \
            | sudo tee /etc/sysctl.d/60-openamr-dds.conf >/dev/null
    fi
    run sudo sysctl --system
}

# ---- step 9: robot service files (install/systemd) -------------------------------------------
check_services() {
    local f
    for f in openamr-agent.service openamr-bringup.service openamr-ui.service openamr.target; do
        cmp -s "$HERE/../systemd/$f" "/etc/systemd/system/$f" || return 1
    done
    [ -x /opt/openamr/bin/openamr-run.sh ] && [ -f /etc/openamr/openamr.env ]
}
do_services() {
    run sudo id -u openamr >/dev/null 2>&1 || run sudo useradd --system --create-home --groups dialout,video openamr
    run sudo install -d /etc/openamr /opt/openamr/bin /data/maps
    run sudo install -m 0755 "$HERE/../systemd/bin/openamr-run.sh" /opt/openamr/bin/openamr-run.sh
    [ -f /etc/openamr/openamr.env ] || run sudo install -m 0644 "$HERE/../systemd/openamr.env" /etc/openamr/openamr.env
    run sudo install -m 0644 "$HERE/../systemd/cyclonedds.xml" /etc/openamr/cyclonedds.xml
    run sudo install -m 0644 "$HERE"/../systemd/openamr*.service "$HERE/../systemd/openamr.target" /etc/systemd/system/
    run sudo systemctl daemon-reload
    log "Services are installed but NOT enabled. Enable after the first bench run."
}

# ---- step 10: workspace build -----------------------------------------------------------------
check_workspace() { [ -f /opt/openamr/ros2/install/setup.bash ]; }
do_workspace() {
    log "Build the workspace: cd <repo>/ros2 && rosdep install --from-paths src -y --ignore-src"
    log "  && colcon build --install-base /opt/openamr/ros2/install (bounded: check free disk first)"
    return 1
}

# ---- step 11: evidence record -----------------------------------------------------------------
check_evidence() { [ -s "$STATE_DIR/install-evidence.txt" ]; }
do_evidence() {
    run sudo install -d "$STATE_DIR"
    if [ "$DRY_RUN" = 0 ]; then
        {
            echo "date: $(date -u +%FT%TZ)"
            echo "script_sha256: $(sha256sum "${BASH_SOURCE[0]}" | cut -d' ' -f1)"
            echo "nv_tegra_release: $(head -1 "$NV_TEGRA_RELEASE_FILE" 2>/dev/null)"
            echo "ubuntu: $(grep PRETTY_NAME "$OS_RELEASE_FILE" 2>/dev/null)"
            echo "kernel: $(uname -r)"
            echo "carrier_sku: TODO(Karthikeyan)"
            echo "ros_domain_id: ${ROS_DOMAIN_ID:-unset}"
            echo "rmw: ${RMW_IMPLEMENTATION:-unset}"
            dpkg-query -W 'ros-jazzy-*' 2>/dev/null | head -200
            chronyc tracking 2>&1 | head -5
        } | sudo tee "$STATE_DIR/install-evidence.txt" >/dev/null
    fi
}

STEPS=(identity ros_apt ros_packages microros_agent udev chrony can_modules sysctl services workspace evidence)

usage() { sed -n '2,18p' "${BASH_SOURCE[0]}"; }

do_step() {
    local s="$1" mode="$2"
    if "check_$s"; then log "[ok]    $s"; return 0; fi
    if [ "$mode" = check ]; then log "[FAIL]  $s"; return 1; fi
    log "[do]    $s"
    "do_$s" || { log "[BLOCKED] $s needs a manual step (see above)"; return 1; }
    if [ "$DRY_RUN" = 1 ]; then return 0; fi
    if "check_$s"; then log "[ok]    $s (verified after the step)"; return 0; fi
    log "[FAIL]  $s did not verify after the step"
    return 1
}

main() {
    local mode="" only=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --dry-run) DRY_RUN=1 ;;
            list) printf '%s\n' "${STEPS[@]}"; return 0 ;;
            check|run) mode="$1" ;;
            -h|--help) usage; return 0 ;;
            *) only="$1" ;;
        esac
        shift
    done
    [ -n "$mode" ] || { usage; return 2; }
    local failed=0 s
    for s in "${STEPS[@]}"; do
        [ -z "$only" ] || [ "$only" = "$s" ] || continue
        do_step "$s" "$mode" || { failed=1; [ "$mode" = run ] && break; }
    done
    return "$failed"
}

main "$@"
