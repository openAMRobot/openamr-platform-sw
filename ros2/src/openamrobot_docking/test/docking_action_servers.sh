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

if [[ "${1:-}" != "--self-test-parse" && "${1:-}" != "--parse-count" \
   && "${1:-}" != "--preflight-only" && "${1:-}" != "--self-test-preflight" ]]; then
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
fi

LOG="${TMPDIR:-/tmp}/docking_harness_$$.log"   # created lazily before the launch; keeps --parse-count side-effect free
TIMEOUT_S=${HARNESS_TIMEOUT_S:-150}
ACTIONS=(/dock_robot /undock_robot)

# `timeout` is coreutils: present on the hosted runner, absent on macOS. Without a bound the query
# can hang the harness forever, which breaks the one promise this script makes — every unanswerable
# query is UNKNOWN, never a wait. So the fallback carries its own deadline rather than running bare.
QUERY_TIMEOUT_S=${QUERY_TIMEOUT_S:-10}
_run() {
  local pid watch rc t=$QUERY_TIMEOUT_S
  if command -v timeout >/dev/null 2>&1; then timeout "$t" "$@"; return $?; fi
  if command -v gtimeout >/dev/null 2>&1; then gtimeout "$t" "$@"; return $?; fi
  # The command gets its own process group (`set -m`), and the deadline kills the group. Killing
  # only the parent leaves grandchildren alive — and a grandchild still holding stdout keeps the
  # caller's command substitution open, which is a hang by another name.
  set -m
  "$@" &
  pid=$!
  ( sleep "$t"; kill -TERM -"$pid" 2>/dev/null; sleep 1; kill -9 -"$pid" 2>/dev/null ) &
  watch=$!
  wait "$pid"
  rc=$?
  kill -TERM -"$watch" 2>/dev/null
  kill -9 -"$watch" 2>/dev/null
  wait "$watch" 2>/dev/null
  set +m
  return $rc
}

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill -INT -"$LAUNCH_PID" 2>/dev/null
    sleep 3
    kill -9 -"$LAUNCH_PID" 2>/dev/null
  fi
  # No pkill: this harness must never signal a process it did not start. Another ROS stack on the
  # same host is none of its business. The process group above is the only thing it owns.
  sleep 2
}
trap cleanup EXIT

# Parse an action-info result into a count, or the literal string UNKNOWN.
#
# Fails closed on all three shapes: a failing command, output with no "Action servers:" line, and a
# line whose value is not a bare non-negative integer. The last case is what makes a truncated or
# reworded upstream output unmeasurable instead of accidentally matching an expected zero/mismatch.
parse_count() {
  local out="$1" rc="$2" n
  if [[ "${rc:-1}" -ne 0 ]]; then
    echo "UNKNOWN"
    return
  fi
  if ! printf '%s\n' "$out" | grep -q '^Action servers:'; then
    echo "UNKNOWN"
    return
  fi
  n=$(printf '%s\n' "$out" | sed -n 's/^Action servers: \([0-9]\{1,\}\)[[:space:]]*$/\1/p' | head -1)
  if [[ -z "$n" ]]; then
    echo "UNKNOWN"
    return
  fi
  echo "$n"
}

query_count() {
  local action="$1" out rc
  out=$(_run ros2 action info "$action" -t 2>&1)
  rc=$?
  parse_count "$out" "$rc"
}

# Direct parser probe: --parse-count "<action info output>" <exit code>. Used by
# test_docking_action_servers_parse.py so the two rejection cases are regressions in the suite.
if [[ "${1:-}" == "--parse-count" ]]; then
  parse_count "${2:-}" "${3:-1}"
  exit 0
fi

# Regression mode: no ROS, no launch. Exercises the parser against the shapes that must not be
# read as a count. Wired into pytest as test_docking_action_servers_parse.py.
if [[ "${1:-}" == "--self-test-parse" ]]; then
  fail=0
  check() {
    local name="$1" want="$2" out="$3" rc="$4" got
    got=$(parse_count "$out" "$rc")
    if [[ "$got" == "$want" ]]; then
      echo "ok   ${name}: ${got}"
    else
      echo "FAIL ${name}: want ${want}, got ${got}"
      fail=1
    fi
  }
  check "well-formed count"        "1"       "Action servers: 1"  0
  check "zero is a real count"     "0"       "Action servers: 0"  0
  check "non-numeric value"        "UNKNOWN" "Action servers: x"  0
  check "missing count field"      "UNKNOWN" "Action servers:"    0
  check "line absent entirely"     "UNKNOWN" "No action servers here" 0
  check "command failed"           "UNKNOWN" "Action servers: 1"  1
  check "empty output"             "UNKNOWN" ""                    0
  exit $fail
fi


# Pre-start check. Refuse to start beside another docking stack, and refuse to start when the graph
# cannot be queried at all: a failed query is UNKNOWN, which is not the same observation as zero.
# Detection only — this harness never signals a process it did not start.
preflight() {
  local action pre
  for action in "${ACTIONS[@]}"; do
    pre=$(query_count "$action")
    if [[ "$pre" == "UNKNOWN" ]]; then
      echo "ERROR: could not query ${action} before the run — UNKNOWN, not zero."
      echo "       A failed graph query is not proof that no server exists."
      return $EXIT_ERROR
    fi
    if [[ "$pre" != "0" ]]; then
      echo "ERROR: ${action} already advertises ${pre} server(s) before this run started."
      echo "       Another docking stack is running on this graph; refusing to measure against it."
      return $EXIT_ERROR
    fi
  done
  return 0
}

# Regression modes for the preflight decision. No ROS needed: a stub `ros2` on PATH produces the
# three shapes that must be told apart. Wired into pytest as test_docking_action_servers_preflight.py.
if [[ "${1:-}" == "--preflight-only" ]]; then
  shift
  [[ $# -gt 0 ]] && ACTIONS=("$@")
  preflight
  exit $?
fi

if [[ "${1:-}" == "--self-test-preflight" ]]; then
  fail=0
  run_case() {
    local name="$1" want="$2" stub="$3" envs="${4:-}" rc
    local dir
    dir=$(mktemp -d)
    mkdir -p "$dir/bin"
    printf '#!/bin/sh\n%s\n' "$stub" >"$dir/bin/ros2"
    chmod +x "$dir/bin/ros2"
    env PATH="$dir/bin:$PATH" $envs bash "${BASH_SOURCE[0]}" --preflight-only /dock_robot /undock_robot >/dev/null 2>&1
    rc=$?
    rm -rf "$dir"
    if [[ "$rc" == "$want" ]]; then
      echo "ok   ${name}: exit ${rc}"
    else
      echo "FAIL ${name}: want exit ${want}, got ${rc}"
      fail=1
    fi
  }
  run_case "graph queryable, no servers"   0  'echo "Action servers: 0"; exit 0'
  run_case "query fails (exit 1)"          20 'exit 1'
  run_case "query returns prose only"      20 'echo "unknown action"; exit 0'
  run_case "foreign stack present"         20 'echo "Action servers: 2"; exit 0'
  # A query that never answers must still end the run: the fallback deadline kills it and the parser
  # reports UNKNOWN. Without this the macOS path blocks forever (review P2, 12 Nov).
  run_case "hung query"                    20 'sleep 300' "QUERY_TIMEOUT_S=2"
  exit $fail
fi
if [[ "$EXPECT" == "0" && -z "$CONTROL_NODE" ]]; then
  echo "ERROR: expecting zero servers requires --control-node, so 'no servers' cannot mean 'no graph'"
  exit $EXIT_ERROR
fi

preflight || exit $?

: >"$LOG"
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
    nodes=$(_run ros2 node list 2>/dev/null || true)
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
    _run ros2 lifecycle get "$node" 2>&1 | tail -1
  done
  echo "--- launch log tail ---"
  tail -60 "$LOG"
  exit $EXIT_ERROR
fi

echo "MISMATCH: timed out after ${TIMEOUT_S}s; the graph answered but the counts are not ${EXPECT}"
echo "--- launch log tail ---"
tail -20 "$LOG"
exit $EXIT_MISMATCH
