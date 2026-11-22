#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
#
# bench_common.sh: helpers sourced by the read-only bench scripts in tools/bench/.
# Nothing in this file sends a message, a CAN frame or a service request.

# Output folder: <base>/<name>-<UTC timestamp>; created once per script run.
bench_init() {
    local name=$1 base=${2:-bench_out}
    BENCH_TS=$(date -u +%Y%m%dT%H%M%SZ)
    BENCH_OUT="${base}/${name}-${BENCH_TS}"
    mkdir -p "$BENCH_OUT"
    BENCH_LOG="${BENCH_OUT}/summary.log"
    BENCH_NOT_CHECKED=()
    : >"$BENCH_LOG"
    say "${name} started ${BENCH_TS} on $(hostname)"
    say "output folder: ${BENCH_OUT}"
    say "read-only: this script commands no motion and sends no CAN frame"
}

say() {
    printf '%s\n' "$*" | tee -a "$BENCH_LOG"
}

section() {
    say ""
    say "== $*"
}

# step <file> <seconds> <command...>
# Runs one bounded command, writes its output to <file> in the output folder and to the
# summary, and records the exit code. A failing or timed-out command never stops the script;
# exit 124 is a timeout (expected for listeners such as `ros2 topic hz`).
step() {
    local file=$1 secs=$2
    shift 2
    local rc=0
    say "+ $* (timeout ${secs}s) > ${file}"
    timeout -s INT -k 5 "$secs" "$@" >"${BENCH_OUT}/${file}" 2>&1 </dev/null || rc=$?
    tee -a "$BENCH_LOG" <"${BENCH_OUT}/${file}"
    say "  exit ${rc}"
    return 0
}

have() {
    command -v "$1" >/dev/null 2>&1
}

not_checked() {
    BENCH_NOT_CHECKED+=("$*")
}

bench_finish() {
    section "Not checked by this script"
    local item
    for item in "${BENCH_NOT_CHECKED[@]}"; do
        say "- ${item}"
    done
    say ""
    say "done; attach ${BENCH_OUT} to the bench record (it is development evidence, not acceptance)"
}
