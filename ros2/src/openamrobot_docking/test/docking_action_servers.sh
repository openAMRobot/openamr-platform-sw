#!/usr/bin/env bash
# Hosted harness: does the docking launch actually bring the action servers up?
#
# The AST test (test_docking_launch_remap.py) proves the launch DECLARES the server node with
# the remap. It cannot prove the lifecycle node activates and that /dock_robot and
# /undock_robot become reachable — which is what platform-sw#125 review asked for.
#
# Usage: docking_action_servers.sh <package> <launch-file> <expected-servers-per-action> [extra launch args...]
#   expected-servers-per-action = 1  -> both actions must advertise exactly one server
#                                 0  -> neither action may advertise a server (Build 1 real profile
#                                       under the "disable the server" D-19 option)
#
# The assertion is a parameter on purpose: the D-19 enforcement decision changes the expected
# number, not the harness.
set -uo pipefail

PKG="${1:?package required}"
LAUNCH_FILE="${2:?launch file required}"
EXPECT="${3:?expected servers per action required}"
shift 3
EXTRA_ARGS=("$@")

LOG=$(mktemp /tmp/docking_harness_XXXX.log)
TIMEOUT_S=150
ACTIONS=(/dock_robot /undock_robot)

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null
    sleep 3
    kill -9 "$LAUNCH_PID" 2>/dev/null
  fi
}
trap cleanup EXIT

echo "launch: ros2 launch ${PKG} ${LAUNCH_FILE} ${EXTRA_ARGS[*]:-} (expect ${EXPECT} server(s) per action)"
ros2 launch "$PKG" "$LAUNCH_FILE" "${EXTRA_ARGS[@]}" >"$LOG" 2>&1 &
LAUNCH_PID=$!

deadline=$((SECONDS + TIMEOUT_S))
declare -A seen=()
while (( SECONDS < deadline )); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "FAIL: launch process exited before the servers appeared"
    tail -40 "$LOG"
    exit 1
  fi
  all_match=1
  for action in "${ACTIONS[@]}"; do
    info=$(timeout 10 ros2 action info "$action" -t 2>/dev/null || true)
    count=$(printf '%s\n' "$info" | sed -n 's/^Action servers: \([0-9]\+\).*/\1/p' | head -1)
    count=${count:-0}
    seen[$action]=$count
    [[ "$count" == "$EXPECT" ]] || all_match=0
  done
  if [[ "$all_match" == "1" ]]; then
    echo "PASS: every action advertises ${EXPECT} server(s)"
    for action in "${ACTIONS[@]}"; do echo "  ${action}: ${seen[$action]}"; done
    exit 0
  fi
  sleep 5
done

echo "FAIL: timed out after ${TIMEOUT_S}s waiting for ${EXPECT} server(s) per action"
for action in "${ACTIONS[@]}"; do echo "  ${action}: ${seen[$action]:-0}"; done
echo "--- lifecycle state ---"
for node in /docking_server /lifecycle_manager_docking; do
  printf '%s: ' "$node"
  timeout 10 ros2 lifecycle get "$node" 2>&1 | tail -1
done
echo "--- launch log tail ---"
tail -60 "$LOG"
exit 1
