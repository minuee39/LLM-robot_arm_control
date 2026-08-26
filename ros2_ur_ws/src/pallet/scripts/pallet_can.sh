#!/usr/bin/env bash

set -euo pipefail

can_interface="${PALLET_CAN_INTERFACE:-can0}"
can_bitrate="${PALLET_CAN_BITRATE:-1000000}"

if ! command -v ip >/dev/null 2>&1; then
  echo "error: the 'ip' command is not installed" >&2
  exit 1
fi

if ! ip link show "${can_interface}" >/dev/null 2>&1; then
  echo "error: CAN interface '${can_interface}' was not found" >&2
  echo "Connect the CAN adapter, then check available interfaces with: ip -brief link" >&2
  exit 1
fi

sudo ip link set dev "${can_interface}" down
sudo ip link set dev "${can_interface}" type can bitrate "${can_bitrate}"
sudo ip link set dev "${can_interface}" up

echo "Pallet CAN ready: interface=${can_interface}, bitrate=${can_bitrate} bit/s"
ip -details link show "${can_interface}"
