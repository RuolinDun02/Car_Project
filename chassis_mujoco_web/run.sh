#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
# Prefer robot env (has flask); no mujoco required anymore
PY="${PY:-/home/jiayi/miniconda3/envs/robot/bin/python}"
export CHASSIS_CAN="${CHASSIS_CAN:-can0}"
export CHASSIS_WEB_PORT="${CHASSIS_WEB_PORT:-8089}"
CAN_BITRATE="${CHASSIS_CAN_BITRATE:-500000}"

bring_up_can() {
  local iface="$1"
  local bitrate="$2"
  if ! ip link show "$iface" &>/dev/null; then
    echo "[run] CAN iface '$iface' not found" >&2
    return 1
  fi
  # Recreate link params even if already UP (bitrate may be wrong)
  sudo ip link set "$iface" down || true
  sudo ip link set "$iface" type can bitrate "$bitrate"
  sudo ip link set "$iface" up
  # Prefer a deeper TX queue so 50Hz multi-frame cmds don't block
  sudo ip link set "$iface" txqueuelen 1000 || true
  echo "[run] $iface UP bitrate=$bitrate"
  ip -brief link show "$iface"
}

bring_up_can "$CHASSIS_CAN" "$CAN_BITRATE"

cd "$ROOT"
exec "$PY" "$ROOT/server.py"
