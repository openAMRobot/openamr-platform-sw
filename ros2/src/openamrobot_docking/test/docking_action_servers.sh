#!/usr/bin/env bash
# Hosted harness: does the docking launch actually bring the action servers up?
#
# The AST test (test_docking_launch_remap.py) proves the launch DECLARES the server node with
# the remap. It cannot prove the lifecycle node activates and that /dock_robot and
# /undock_robot become reachable — which is what platform-sw#125 review asked for.
#
# Usage:
#   docking_action_servers.sh <package> <launch-file> <expected-servers>
#                             [--control-node NAME] -- [extra launch args...]
#
#   expected-servers = 1  -> both actions must advertise exactly one server
#                    = 0  -> neither action may advertise a server (a profile that disables it)
#
# Exit codes are distinct on purpose, so a caller can accept "counts differ" without also
# accepting "the graph never came up":
#   0   PASS   — every action reported exactly the expected count, control node visible
#   10  MISMATCH — the graph was healthy and answered; the counts differ from the expectation
#                (this is the only code a negative control may treat as success)
#   20  ERROR  — infrastructure: launch exited, a query stayed UNKNOWN, leftover servers at
#                start, or the required control node never appeared. Proves nothing either way.
#
# Fails closed. A query that errors, or output without an "Action servers:" line, is UNKNOWN and
# fails the run — it is never read as zero. Expecting zero requires a control node, so
# "no servers" cannot be the same observation as "no graph".
#
# The launch is started in its own session and torn down by process group: `ros2 launch` is a
# supervisor, so killing only the parent leaves the node processes behind. An orphaned
# docking_server from an earlier step is exactly how the first negative control passed with the
# server switched off.
set -uo pipefail

EXIT_PASS=0
EXIT_MISMATCH=10
EXIT_ERROR=20

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
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT -"$LAUNCH_PID" 2>/dev/null
    sleep 3
    kill -9 -"$LAUNCH_PID" 2>/dev/null
  fi
  pkill -9 -f 'opennav_docking' 2>/dev/null
  pkill -9 -f 'lifecycle_manager_docking' 2>/dev/null
  sleep 2
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

if [[ "$EXPECT" == "0" && -z "$CONTROL_NODE" ]]; then
  echo "ERROR: expecting zero servers requires --control-node, so 'no servers' cannot mean 'no graph'"
  exit $EXIT_ERROR
fi

# Nothing left over from an earlier step may still be advertising these actions.
pkill -9 -f 'opennav_docking' 2>/dev/null
sleep 2
for action in "${ACTIONS[@]}"; do
  pre=$(timeout 10 ros2 action info "$action" -t 2>/dev/null | sed -n 's/^Action servers: \([0-9]\+\).*/\1/p' | head -1)
  if [[ "${pre:-0}" != "0" ]]; then
    echo "ERROR: ${action} already advertises ${pre} server(s) before this run started"
    exit $EXIT_ERROR
  fi
done

echo "launch: ros2 launch ${PKG} ${LAUNCH_FILE} ${EXTRA_ARGS[*]:-} (expect ${EXPECT} server(s) per action)"
LAUNCH_PID=$(setsid ros2 launch "$PKG" "$LAUNCH_FILE" "${EXTRA_ARGS[@]}" >"$LOG" 2>&1 & echo $!)

deadline=$((SECONDS + TIMEOUT_S))
declare -A seen=()
control_seen=0
while (( SECONDS < deadline )); do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "ERROR: launch process exited before the servers settled"
    tail -40 "$LOG"
    exit $EXIT_ERROR
  fi

  # Positive control: the graph must be queryable and must contain a node we know the launch
  # started, before any server count — especially a zero — is believed.
  control_ok=1
  if [[ -n "$CONTROL_NODE" ]]; then
    nodes=$(timeout 10 ros2 node list 2>/dev/null || true)
    if printf '%s\n' "$nodes" | grep -qx "/${CONTROL_NODE}"; then
      control_seen=1
    else
      control_ok=0
    fi
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
    exit $EXIT_PASS
  elif [[ "$all_match" == "1" && "$control_ok" == "0" ]]; then
    echo "waiting: counts match but the control node /${CONTROL_NODE} is not visible"
  fi
  sleep 5
done

for action in "${ACTIONS[@]}"; do echo "  ${action}: ${seen[$action]:-UNKNOWN}"; done
echo "control node /${CONTROL_NODE:-<none>} seen during run: ${control_seen}"

# Distinguish "healthy graph, wrong counts" from "could not measure".
unknown_final=0
for action in "${ACTIONS[@]}"; do
  [[ "${seen[$action]:-UNKNOWN}" == "UNKNOWN" ]] && unknown_final=1
done
if [[ "$unknown_final" == "1" || ( -n "$CONTROL_NODE" && "$control_seen" != "1" ) ]]; then
  echo "ERROR: timed out after ${TIMEOUT_S}s without a healthy, observable graph — this proves nothing"
  echo "--- lifecycle state ---"
  for node in /docking_server /lifecycle_manager_docking; do
    printf '%s: ' "$node"
    timeout 10 ros2 lifecycle get "$node" 2>&1 | tail -1
  done
  echo "--- launch log tail ---"
  tail -60 "$LOG"
  exit $EXIT_ERROR
fi

echo "MISMATCH: timed out after ${TIMEOUT_S}s; the graph answered but the counts are not ${EXPECT}"
echo "--- launch log tail ---"
tail -20 "$LOG"
exit $EXIT_MISMATCH
