#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
#
# ros_graph_snapshot.sh: record the ROS 2 graph of the robot into a timestamped folder.
# Read-only. Commands no robot motion, publishes nothing, calls no service.
#
# Records: ROS_DOMAIN_ID, RMW and discovery variables, network interfaces and routes, nodes,
# every topic with its type and publisher/subscriber counts, every publisher and subscriber of
# each velocity command topic (any topic whose name contains cmd_vel), the TF tree
# (tf2_tools view_frames), and bounded rates of the odometry topics and /scan.
#
# Usage: ros_graph_snapshot.sh [--seconds N] [--out DIR]
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=tools/bench/bench_common.sh
. "${here}/bench_common.sh"

# Topic names from openamrobot_bringup (real_bringup.launch.py, config/ekf.yaml).
RATE_TOPICS=(/odom /odom/unfiltered /scan /imu/data)

usage() {
    echo "usage: $0 [--seconds N] [--out DIR]" >&2
    exit 2
}

seconds=10
out=bench_out
while [ $# -gt 0 ]; do
    case $1 in
        --seconds) seconds=${2:?missing value}; shift 2 ;;
        --out) out=${2:?missing value}; shift 2 ;;
        *) usage ;;
    esac
done
case $seconds in
    '' | *[!0-9]*) echo "--seconds must be a whole number" >&2; exit 2 ;;
esac
[ "$seconds" -ge 3 ] && [ "$seconds" -le 60 ] || { echo "--seconds must be 3 to 60" >&2; exit 2; }

bench_init ros_graph_snapshot "$out"

section "1 Environment and network"
step env.txt 5 sh -c "env | grep -E '^(ROS_|RMW_|CYCLONEDDS_|FASTRTPS_|FASTDDS_)' | sort || echo 'no ROS variables set'"
say "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset (0)} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset (rmw_fastrtps_cpp default)}"
if have ip; then
    step ip_addr.txt 5 ip -brief address
    step ip_route.txt 5 ip route
else
    not_checked "network interfaces: ip (iproute2) not installed"
fi

if ! have ros2; then
    say "BLOCKED: ros2 not on PATH (source /opt/ros/jazzy/setup.bash and the workspace)"
    not_checked "every ROS graph check: ros2 not on PATH"
    bench_finish
    exit 0
fi

section "2 Nodes and topics"
step nodes.txt 15 ros2 node list
step topics.txt 15 ros2 topic list -t
counts="${BENCH_OUT}/topic_counts.tsv"
printf 'topic\ttype\tpublishers\tsubscribers\n' >"$counts"
while read -r topic type; do
    [ -n "$topic" ] || continue
    info=$(timeout -s INT -k 3 10 ros2 topic info "$topic" 2>&1 </dev/null || true)
    pubs=$(sed -n 's/^Publisher count: //p' <<<"$info")
    subs=$(sed -n 's/^Subscription count: //p' <<<"$info")
    printf '%s\t%s\t%s\t%s\n' "$topic" "$type" "${pubs:-?}" "${subs:-?}" >>"$counts"
done <"${BENCH_OUT}/topics.txt"
say "+ ros2 topic info <each topic> > topic_counts.tsv"
tee -a "$BENCH_LOG" <"$counts"
awk -F'\t' 'NR > 1 && $3 == "0" { print "no publisher: " $1 }' "$counts" | tee -a "$BENCH_LOG"

section "3 Velocity command topics: who publishes and who listens"
mapfile -t vel_topics < <(awk '{ print $1 }' "${BENCH_OUT}/topics.txt" | grep cmd_vel || true)
if [ "${#vel_topics[@]}" -eq 0 ]; then
    say "no topic with cmd_vel in its name"
fi
i=0
for t in "${vel_topics[@]}"; do
    i=$((i + 1))
    step "vel_topic_${i}.txt" 10 ros2 topic info -v "$t"
done
not_checked "which velocity command topic the base actually obeys (gate (d), contract request #170)"

section "4 TF tree"
if ros2 pkg prefix tf2_tools >/dev/null 2>&1; then
    # view_frames writes frames_<date>.gv and .pdf into the working directory.
    step tf_view_frames.txt 30 sh -c "cd '${BENCH_OUT}' && ros2 run tf2_tools view_frames"
else
    say "tf2_tools not installed (ros-jazzy-tf2-tools); recording /tf_static once instead"
    not_checked "TF tree drawing: tf2_tools not installed"
fi
step tf_static.txt 10 ros2 topic echo --once /tf_static

section "5 Rates (bounded ${seconds}s each; exit 124 is the bound, not an error)"
for t in "${RATE_TOPICS[@]}"; do
    if awk '{ print $1 }' "${BENCH_OUT}/topics.txt" | grep -qx -- "$t"; then
        step "hz_$(tr '/' '_' <<<"${t#/}").txt" "$seconds" ros2 topic hz "$t"
    else
        say "${t}: not in the graph"
    fi
done

section "6 Daemon and middleware"
step ros2_doctor.txt 30 ros2 doctor --report

not_checked "QoS compatibility per endpoint beyond what topic_counts and vel_topic_*.txt show"
not_checked "clock source and use_sim_time per node (ros2 param get per node, not run here)"
not_checked "rates against their requirement; this script records them only"
not_checked "a second host's view of the graph: run this script on each host"
bench_finish
