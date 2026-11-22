#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 OpenAMRobot (Botshare LTD)
#
# can_readonly.sh: read the state of the SocketCAN interfaces and listen for a bounded time.
# Read-only. Sends no CAN frame, changes no link setting (no bitrate, no up/down, no
# ctrlmode), configures no drive and commands no motion.
#
# Records per interface: `ip -details -statistics link show`, state, bitrate, sample point,
# controller mode (whether LISTEN-ONLY is on), bus error counters, restarts, kernel counters,
# then a bounded candump capture with error frames, and a per-ID frame count.
#
# Note: candump itself transmits nothing, but a CAN controller that is not in listen-only
# mode still acknowledges frames on the bus in hardware. This script does not switch the
# mode (that needs the link taken down); it reports it.
#
# Usage: can_readonly.sh [--iface NAME]... [--seconds N] [--out DIR]
# Any other argument is refused, so nothing can be passed through to a sending tool.
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=tools/bench/bench_common.sh
. "${here}/bench_common.sh"

refuse() {
    echo "REFUSED: $*" >&2
    echo "usage: $0 [--iface NAME]... [--seconds N] [--out DIR] (listen only)" >&2
    exit 2
}

# Refuse before parsing anything that looks like a frame, a sending tool or a link change.
for a in "$@"; do
    case $a in
        *'#'*) refuse "argument '$a' looks like a CAN frame" ;;
        *send* | *gen* | *player* | *bcm* | *isotp* | *xmit* | *tx*) refuse "argument '$a' names a sending tool or mode" ;;
        *bitrate* | *restart* | *ctrlmode* | *listen-only* | up | down | set) refuse "argument '$a' would change the link" ;;
    esac
done

ifaces=()
seconds=10
out=bench_out
while [ $# -gt 0 ]; do
    case $1 in
        --iface) ifaces+=("${2:?missing value}"); shift 2 ;;
        --seconds) seconds=${2:?missing value}; shift 2 ;;
        --out) out=${2:?missing value}; shift 2 ;;
        *) refuse "unknown argument '$1'" ;;
    esac
done
for n in "${ifaces[@]}"; do
    [[ $n =~ ^[A-Za-z0-9_.-]{1,15}$ ]] || refuse "interface name '$n' is not a plain name"
done
case $seconds in
    '' | *[!0-9]*) refuse "--seconds must be a whole number" ;;
esac
[ "$seconds" -ge 1 ] && [ "$seconds" -le 120 ] || refuse "--seconds must be 1 to 120"

bench_init can_readonly "$out"
have ip || { say "BLOCKED: ip (iproute2) not installed"; not_checked "everything: no ip tool"; bench_finish; exit 0; }

section "1 CAN interfaces"
step links_can.txt 5 ip -details -statistics link show type can
step links_vcan.txt 5 ip -details -statistics link show type vcan
if [ "${#ifaces[@]}" -eq 0 ]; then
    mapfile -t ifaces < <(ip -o link show type can 2>/dev/null | awk -F': ' '{ sub(/@.*/, "", $2); print $2 }')
    say "interfaces found: ${ifaces[*]:-none}"
fi
if [ "${#ifaces[@]}" -eq 0 ]; then
    say "no CAN interface (adapter not connected, driver not loaded, or the name differs)"
    not_checked "bitrate, error counters and traffic: no CAN interface"
fi

have candump || not_checked "traffic capture: candump not installed (can-utils)"

for n in "${ifaces[@]}"; do
    section "2 ${n}"
    if [ ! -e "/sys/class/net/${n}" ]; then
        say "${n}: no such interface"
        not_checked "${n}: interface missing"
        continue
    fi
    step "${n}_details.txt" 5 ip -details -statistics link show dev "$n"
    d="${BENCH_OUT}/${n}_details.txt"
    say "operstate: $(cat "/sys/class/net/${n}/operstate")"
    say "can state: $(grep -oE 'can (<[^>]*> )?state [A-Z-]+' "$d" || echo 'not reported (vcan has none)')"
    say "bitrate:   $(grep -oE 'bitrate [0-9]+ sample-point [0-9.]+' "$d" | head -1 || echo 'not reported')"
    say "berr:      $(grep -oE 'berr-counter tx [0-9]+ rx [0-9]+' "$d" || echo 'not reported')"
    say "restarts:  $(grep -oE 'restart-ms [0-9]+' "$d" || echo 'restart-ms not reported')"
    if grep -q 'LISTEN-ONLY' "$d"; then
        say "mode: LISTEN-ONLY on (the controller does not acknowledge frames)"
    else
        say "mode: LISTEN-ONLY off (the controller acknowledges frames; candump sends none)"
    fi
    step "${n}_kernel_counters.txt" 5 sh -c "cd /sys/class/net/${n}/statistics && grep -H . rx_packets tx_packets rx_errors tx_errors rx_dropped tx_dropped"
    if ! have candump; then
        continue
    fi
    if [ "$(cat "/sys/class/net/${n}/operstate")" = "down" ]; then
        say "${n} is down; not brought up by this script"
        not_checked "${n}: traffic (interface down)"
        continue
    fi
    # All data frames (0:0) and all error frames (#FFFFFFFF), absolute time, RX/TX flag.
    step "${n}_candump.txt" "$seconds" candump -t a -x -e "${n},0:0,#FFFFFFFF"
    say "frames captured: $(grep -c "^ *([0-9]" "${BENCH_OUT}/${n}_candump.txt" || true)"
    say "per CAN ID (count id):"
    awk '$1 ~ /^\(/ { for (i = 2; i <= NF; i++) if ($i ~ /^[0-9A-F]+$/ && length($i) <= 8 && $(i+1) ~ /^\[/) { print $i; break } }' \
        "${BENCH_OUT}/${n}_candump.txt" | sort | uniq -c | sort -rn | tee -a "$BENCH_LOG"
    if grep -q ' TX ' "${BENCH_OUT}/${n}_candump.txt"; then
        say "note: TX lines are frames sent by another process on this host, not by this script"
    fi
done

not_checked "bus termination (about 60 ohm per bus, measured power off)"
not_checked "that each bitrate matches the node on that bus (compare with the firmware and drive owner)"
not_checked "which node sends which ID: map the IDs against the CANopen node IDs by hand"
not_checked "true passive listening when LISTEN-ONLY is off (the controller ACKs)"
bench_finish
