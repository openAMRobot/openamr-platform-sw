#!/usr/bin/env bash
# Hosted harness: does the docking launch actually bring the action servers up?
#
# The AST test (test_docking_launch_remap.py) proves the launch DECLARES the server node with
# the remap. It cannot prove the lifecycle node activates and that /dock_robot and
# /undock_robot become reachable — which is what platform-sw#125 review asked for.
#
# Usage:
#   docking_action_servers.sh <package> <launch-file> <expected-servers> [--control-node NAME] --
#                             [extra launch args...]
#
#   expected-servers = 1  -> both actions must advertise exactly one server
#                    = 0  -> neither action may advertise a server (a profile that disables it)
#
# Fails closed. A query that errors, or output without an "Action servers:" line, is UNKNOWN and
# fails the run — it is never read as zero. When expecting zero, a positive control node must be
# named and visible, so that "no servers" cannot be the same observation as "no graph".
set -uo pipefail

PKG="${1:?package required}"
LAUNCH_FILE="${2:?launch file required}"
EXPECT="${3:?expected servers per action required}"
shift 3

CONTROL_NODE=""
if [[ "${1:-}" == "--control-node" ]]; then
  CONTROL_NODE="${2:?control node name required}"
  shift 2
fi
[[ "${1:-}" == "--" ]] && shift
EXTRA_ARGS=("$@")

LOG=$(mktemp /tmp/docking_harness_XXXX.log)
TIMEOUT_S=${HARNESS_TIMEOUT_S:-150}
ACTIONS=(/dock_robot /undock_robot)

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null
    sleep 3
    kill -9 "$LAUNCH_PID" 2>/dev/null
  fi
}
trap cleanup EXIT

# Echo the server count, or the literal string UNKNOWN. Never converts a failure into a 0.
query_count() {
  local action="$1" out rc
  out=$(timeout 10 ros2 action info "$action" -t 2>&1)
  rc=$?
  if [[ $rc -ne 0 ]]; then
    echo "UNKNOWN"
    return
  fi
  if ! printf '%s\n' "$out" | grep -q '^Action servers:'; then
    echo "UNKNOWN"
    return
  fi
  printf '%s\n' "$out" | sed -n 's/^Action servers: \([0-9]\+\).*/\1/p' | head -1
}

echo "launch: ros2 launch ${PKG} ${LAUNCH_FILE} ${EXTRA_ARGS[*]:-} (expect ${EXPECT} server(s) per action)"
if [[ "$EXPECT" == "0" && -z "$CONTROL_NODE" ]]; then
  echo "FAIL: expecting zero servers requires --control-node, so 'no servers' cannot mean 'no graph'"
  exit 1
fi
ros2 launch "$PKG" "$LAUNCH_FILE" "${EXTRA_ARGS[@]}" >"$LOG" 2>&1 &
LAUNCH_PID=$!

deadline=$((SECONDS + TIMEOUT_S))
declare -A seen=()
while (( SECONDS < deadline )); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "FAIL: launch process exited before the servers settled"
    tail -40 "$LOG"
    exit 1
  fi

  # Positive control: the graph must be queryable and must contain a node we know the launch
  # started, before any server count — especially a zero — is believed.
  control_ok=1
  if [[ -n "$CONTROL_NODE" ]]; then
    nodes=$(timeout 10 ros2 node list 2>/dev/null || true)
    printf '%s\n' "$nodes" | grep -qx "/${CONTROL_NODE}" || control_ok=0
  fi

  all_match=1
  unknown=0
  for action in "${ACTIONS[@]}"; do
    count=$(query_count "$action")
    seen[$action]=$count
    [[ "$count" == "UNKNOWN" ]] && unknown=1
    [[ "$count" == "$EXPECT" ]] || all_match=0
  done

  if [[ "$unknown" == "1" ]]; then
    echo "waiting: at least one action graph query is unavailable (not treated as zero)"
  elif [[ "$all_match" == "1" && "$control_ok" == "1" ]]; then
    echo "PASS: every action advertises ${EXPECT} server(s)${CONTROL_NODE:+ (control node /${CONTROL_NODE} visible)}"
    for action in "${ACTIONS[@]}"; do echo "  ${action}: ${seen[$action]}"; done
    exit 0
  elif [[ "$all_match" == "1" && "$control_ok" == "0" ]]; then
    echo "waiting: counts match but the control node /${CONTROL_NODE} is not visible"
  fi
  sleep 5
done

echo "FAIL: timed out after ${TIMEOUT_S}s waiting for ${EXPECT} server(s) per action"
for action in "${ACTIONS[@]}"; do echo "  ${action}: ${seen[$action]:-UNKNOWN}"; done
echo "control node /${CONTROL_NODE:-<none>} visible: ${control_ok:-n/a}"
echo "--- lifecycle state ---"
for node in /docking_server /lifecycle_manager_docking; do
  printf '%s: ' "$node"
  timeout 10 ros2 lifecycle get "$node" 2>&1 | tail -1
done
echo "--- launch log tail ---"
tail -60 "$LOG"
exit 1
