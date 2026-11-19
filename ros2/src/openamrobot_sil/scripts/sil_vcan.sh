#!/bin/sh
# Create, check or remove the Linux vcan interfaces of the SIL layer.
#   sil_vcan.sh up|check|down IFACE[:fd] [IFACE[:fd] ...]
# IFACE is a classic CAN bus: MTU 16. IFACE:fd is a CAN FD bus: MTU 72. The MTU is always
# set explicitly, never left at the kernel default.
# An interface is accepted only if its name starts with "vcan", its device is under
# /sys/devices/virtual/net/ and its link type is CAN (280). A hardware adapter renamed to a
# vcan name is refused (exit 2).
# Needs root or passwordless sudo for up and down. Exit 3 with a line starting "BLOCKED:" when
# vcan is not available. A blocked run is not a pass.
# The sysfs root is fixed. No environment variable moves it; the tests run a patched copy.
set -u
SYSFS=/sys
CAN_MTU=16
CANFD_MTU=72
ARPHRD_CAN=280
action="${1:-}"
[ $# -ge 2 ] || { echo "usage: $0 up|check|down IFACE[:fd] [IFACE[:fd] ...]" >&2; exit 2; }
shift
if [ "$(id -u)" -eq 0 ]; then SUDO=""; else SUDO="sudo -n"; fi

blocked() { echo "BLOCKED: $*" >&2; exit 3; }
refused() { echo "refused: $*" >&2; exit 2; }

# Refuse an existing interface that is not a virtual CAN interface.
require_virtual_can() {
  link="$SYSFS/class/net/$1"
  [ -e "$link" ] || return 0
  real="$(readlink -f "$link")" || refused "$1: cannot resolve $link"
  virtual="$(readlink -f "$SYSFS/devices/virtual/net")" || refused "$1: no virtual net devices"
  [ "$(dirname "$real")" = "$virtual" ] \
    || refused "$1 is not under /sys/devices/virtual/net/ (a hardware interface)"
  [ "$(cat "$link/type" 2>/dev/null)" = "$ARPHRD_CAN" ] \
    || refused "$1 is not a CAN interface (link type is not $ARPHRD_CAN)"
}

case "$action" in up|check|down) ;; *) echo "unknown action: $action" >&2; exit 2 ;; esac

# Pass 1: validate every argument before anything is changed.
for arg in "$@"; do
  iface="${arg%%:*}"
  [ "$arg" = "$iface" ] || [ "$arg" = "$iface:fd" ] || refused "$arg: the only suffix is :fd"
  case "$iface" in
    vcan*/*|vcan*:*) refused "$iface is not an interface name" ;;
    vcan*) ;;
    *) refused "$iface is not a vcan name" ;;
  esac
  require_virtual_can "$iface"
done

if [ "$action" != check ]; then
  command -v ip >/dev/null 2>&1 || blocked "the ip command is missing (install iproute2)"
fi

for arg in "$@"; do
  iface="${arg%%:*}"
  case "$arg" in *:fd) mtu=$CANFD_MTU ;; *) mtu=$CAN_MTU ;; esac
  case "$action" in
    up)
      if [ ! -e "$SYSFS/class/net/$iface" ]; then
        $SUDO modprobe vcan 2>/dev/null || true
        $SUDO ip link add dev "$iface" type vcan 2>/dev/null \
          || blocked "cannot create $iface (vcan module missing or no permission; on Ubuntu install linux-modules-extra-\$(uname -r))"
        require_virtual_can "$iface"
      fi
      # vcan refuses an MTU change while the interface is up.
      if [ "$(cat "$SYSFS/class/net/$iface/mtu" 2>/dev/null)" != "$mtu" ]; then
        $SUDO ip link set dev "$iface" down || blocked "cannot bring down $iface"
        $SUDO ip link set dev "$iface" mtu "$mtu" || blocked "cannot set MTU $mtu on $iface"
      fi
      $SUDO ip link set dev "$iface" up || blocked "cannot bring up $iface"
      echo "$iface up, mtu $mtu"
      ;;
    check)
      [ -e "$SYSFS/class/net/$iface" ] || blocked "$iface does not exist"
      [ "$(cat "$SYSFS/class/net/$iface/mtu")" = "$mtu" ] \
        || { echo "$iface: MTU is $(cat "$SYSFS/class/net/$iface/mtu"), expected $mtu" >&2; exit 1; }
      echo "$iface present, virtual CAN, mtu $mtu"
      ;;
    down)
      if [ -e "$SYSFS/class/net/$iface" ]; then
        $SUDO ip link delete "$iface" || exit 1
        echo "$iface removed"
      fi
      ;;
  esac
done
