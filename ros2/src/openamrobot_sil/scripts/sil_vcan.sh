#!/bin/sh
# Create, check or remove the Linux vcan interfaces of the SIL layer.
#   sil_vcan.sh up|check|down IFACE [IFACE ...]
# Needs root or passwordless sudo for up and down. Exit 3 with a line starting "BLOCKED:" when
# vcan is not available. A blocked run is not a pass.
set -u
action="${1:-}"
[ $# -ge 2 ] || { echo "usage: $0 up|check|down IFACE [IFACE ...]" >&2; exit 2; }
shift
if [ "$(id -u)" -eq 0 ]; then SUDO=""; else SUDO="sudo -n"; fi

blocked() { echo "BLOCKED: $*" >&2; exit 3; }

command -v ip >/dev/null 2>&1 || blocked "the ip command is missing (install iproute2)"

for iface in "$@"; do
  case "$iface" in
    vcan*) ;;
    *) echo "refused: $iface is not a vcan name" >&2; exit 2 ;;
  esac
  case "$action" in
    up)
      if ! ip link show "$iface" >/dev/null 2>&1; then
        $SUDO modprobe vcan 2>/dev/null || true
        $SUDO ip link add dev "$iface" type vcan 2>/dev/null \
          || blocked "cannot create $iface (vcan module missing or no permission; on Ubuntu install linux-modules-extra-\$(uname -r))"
      fi
      $SUDO ip link set up "$iface" || blocked "cannot bring up $iface"
      echo "$iface up"
      ;;
    check)
      ip link show "$iface" >/dev/null 2>&1 || blocked "$iface does not exist"
      echo "$iface present"
      ;;
    down)
      if ip link show "$iface" >/dev/null 2>&1; then
        $SUDO ip link delete "$iface" || exit 1
        echo "$iface removed"
      fi
      ;;
    *) echo "unknown action: $action" >&2; exit 2 ;;
  esac
done
