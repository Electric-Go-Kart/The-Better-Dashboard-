#!/bin/sh
set -eu

CAN_IFACE="${1:-can0}"
CAN_BITRATE="${CAN_BITRATE:-500000}"
CAN_RESTART_MS="${CAN_RESTART_MS:-100}"

/sbin/ip link set "${CAN_IFACE}" down 2>/dev/null || true
/sbin/ip link set "${CAN_IFACE}" type can bitrate "${CAN_BITRATE}" restart-ms "${CAN_RESTART_MS}"
/sbin/ip link set "${CAN_IFACE}" up
