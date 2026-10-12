#!/usr/bin/env python3
"""FW-mid chassis digital twin web: Gazebo URDF + live CAN → browser 3D."""

from __future__ import annotations

import math
import os
import socket
import struct
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

from flask import Flask, jsonify, request, send_from_directory

ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
ASSETS = ROOT / "assets"

CAN_IFACE = os.environ.get("CHASSIS_CAN", "can0")
HOST = os.environ.get("CHASSIS_WEB_HOST", "0.0.0.0")
PORT = int(os.environ.get("CHASSIS_WEB_PORT", "8089"))

GEAR_NAME = {
    0: "disable",
    1: "驻车",
    2: "空挡",
    4: "FR-",
    6: "4T4D",
    8: "横移",
}

CAN_FMT = "=IB3x8s"
CAN_EFF_FLAG = 0x80000000

# TX CAN IDs (extended; OR EFF when sending)
TX_CTRL = 0x18C4D1D0
TX_STEERING = 0x18C4D2D0
TX_FRONT_VEL = 0x18C4D3D0
TX_REAR_VEL = 0x18C4D4D0
TX_FRONT_ANGLE = 0x18C4D5D0
TX_REAR_ANGLE = 0x18C4D6D0
TX_IO = 0x18C4D7D0
TX_MOTOR = 0x18C4D8D0

MAX_SPEED = float(os.environ.get("CHASSIS_MAX_SPEED", "0.6"))
MAX_WZ = float(os.environ.get("CHASSIS_MAX_WZ", "45"))  # deg/s
CMD_HZ = 50.0
CMD_TIMEOUT = 0.25
# Trapezoidal velocity limits (m/s², deg/s²) — smooth accel / decel
ACCEL_VX = float(os.environ.get("CHASSIS_ACCEL_VX", "0.35"))
DECEL_VX = float(os.environ.get("CHASSIS_DECEL_VX", "0.45"))
ACCEL_WZ = float(os.environ.get("CHASSIS_ACCEL_WZ", "50"))
DECEL_WZ = float(os.environ.get("CHASSIS_DECEL_WZ", "70"))
VEL_EPS = 0.008  # m/s — treat as stopped
WZ_EPS = 0.5  # deg/s

# Official Gazebo URDF joint names
STEER_JOINTS = {
    "lf": "left_steering_hinge_joint",
    "rf": "right_steering_hinge_joint",
    "lr": "rear_left_steering_hinge_joint",
    "rr": "rear_right_steering_hinge_joint",
}
WHEEL_JOINTS = {
    "lf": "left_front_wheel_joint",
    "rf": "right_front_wheel_joint",
    "lr": "left_rear_wheel_joint",
    "rr": "right_rear_wheel_joint",
}


def _s16_nibble(d0: int, d1: int, d2: int) -> int:
    v = ((d2 & 0x0F) << 12) | (d1 << 4) | ((d0 & 0xF0) >> 4)
    return v - 0x10000 if v & 0x8000 else v


def _s16(lo: int, hi: int) -> int:
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v


def _u16(lo: int, hi: int) -> int:
    return (hi << 8) | lo


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _to_s16(v: float) -> int:
    iv = int(round(v))
    if iv > 32767:
        iv = 32767
    if iv < -32768:
        iv = -32768
    return iv


def _pack_gear_s16_s16(gear: int, a: int, b: int, count: int) -> bytes:
    """Pack gear + two signed-16 values in YHS nibble layout (vel/angle free cmds)."""
    a &= 0xFFFF
    b &= 0xFFFF
    d = bytearray(8)
    d[0] = (gear & 0x0F) | ((a & 0x0F) << 4)
    d[1] = (a >> 4) & 0xFF
    d[2] = ((a >> 12) & 0x0F) | ((b & 0x0F) << 4)
    d[3] = (b >> 4) & 0xFF
    d[4] = (b >> 12) & 0x0F
    d[6] = (count & 0x0F) << 4
    d[7] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]
    return bytes(d)


def _pack_ctrl(gear: int, vx_mms: int, wz_cdeg: int, vy_mms: int, count: int) -> bytes:
    x = vx_mms & 0xFFFF
    z = wz_cdeg & 0xFFFF
    y = vy_mms & 0xFFFF
    d = bytearray(8)
    d[0] = (gear & 0x0F) | ((x & 0x0F) << 4)
    d[1] = (x >> 4) & 0xFF
    d[2] = ((x >> 12) & 0x0F) | ((z & 0x0F) << 4)
    d[3] = (z >> 4) & 0xFF
    d[4] = ((z >> 12) & 0x0F) | ((y & 0x0F) << 4)
    d[5] = (y >> 4) & 0xFF
    d[6] = ((y >> 12) & 0x0F) | ((count & 0x0F) << 4)
    d[7] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]
    return bytes(d)


def _pack_motor_enable(enable: bool, count: int) -> bytes:
    """Motor enable bitmap: low4=drive LF/LR/RF/RR, high4=steer LF/LR/RF/RR."""
    d = bytearray(8)
    # 0xFF = 驱动+转向全部使能（与官方 motor_cmd 位定义一致）
    d[0] = 0xFF if enable else 0x00
    d[6] = (count & 0x0F) << 4
    d[7] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]
    return bytes(d)


def _pack_io_unlock(count: int) -> bytes:
    """Minimal IO cmd: unlock pulse bit (matches yhs io_cmd unlock)."""
    d = bytearray(8)
    d[0] = 0x02  # unlock enable
    d[6] = (count & 0x0F) << 4
    d[7] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]
    return bytes(d)


def _pack_io_brake(count: int, brake: bool = True, unlock: bool = False) -> bytes:
    """IO: braking lamp (+ optional unlock). Official io_cmd bit layout."""
    d = bytearray(8)
    if unlock:
        d[0] |= 0x02
    if brake:
        d[0] |= 0x01  # lamp_ctrl
        d[1] |= 0x10  # braking_lamp
    d[6] = (count & 0x0F) << 4
    d[7] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]
    return bytes(d)


def _ramp(cur: float, tgt: float, accel: float, decel: float, dt: float) -> float:
    """Approach tgt with separate accel/decel magnitude limits (trapezoidal)."""
    if dt <= 0.0:
        return cur
    braking = abs(tgt) < abs(cur) - 1e-12 or (cur * tgt < 0.0)
    step = (decel if braking else accel) * dt
    delta = tgt - cur
    if abs(delta) <= step:
        return tgt
    return cur + math.copysign(step, delta)


class CanCommander:
    """50Hz CAN command heartbeat for FW-mid drive / free-wheel control."""

    def __init__(self, can_iface: str, state: "ChassisState") -> None:
        self.can_iface = can_iface
        self.state = state
        self.lock = threading.Lock()
        self.running = True
        self.sock: Optional[socket.socket] = None
        self.mode = "idle"  # 无指令：静默，不自动发 CAN
        self.gear = 6
        # Targets from UI / API
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0  # deg/s
        # Actually transmitted (ramped) setpoints
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.coasting = False  # targets=0, still ramping cmd → then hold
        self.wheel_vel = {"lf": 0.0, "rf": 0.0, "lr": 0.0, "rr": 0.0}
        self.wheel_angle = {"lf": 0.0, "rf": 0.0, "lr": 0.0, "rr": 0.0}
        self.last_cmd_t = time.time()
        self._last_ramp_t = time.time()
        self.tx_ok = False
        self.tx_err = ""
        self._cnt = 0
        self._motor_cnt = 0
        self._io_cnt = 0
        # 仅在有运动指令(drive/free)时周期发使能；无指令保持失能/静默
        self.motor_enable = False
        self._motor_hb_t = 0.0
        self.soft_estop = False  # 软件急停锁存：零速+失能，拒绝运动直至解除
        self._estop_hb_t = 0.0
        self._park_div = 0  # hold 降频计数，避免无反馈时刷爆总线

    def start(self) -> None:
        threading.Thread(target=self._loop, name="can-tx", daemon=True).start()

    def stop(self) -> None:
        self.running = False
        self.set_idle()

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "mode": self.mode,
                "gear": self.gear,
                "vx": self.vx,
                "vy": self.vy,
                "wz": self.wz,
                "cmd_vx": round(self.cmd_vx, 4),
                "cmd_vy": round(self.cmd_vy, 4),
                "cmd_wz": round(self.cmd_wz, 2),
                "coasting": self.coasting,
                "wheel_vel": dict(self.wheel_vel),
                "wheel_angle": dict(self.wheel_angle),
                "tx_ok": self.tx_ok,
                "tx_err": self.tx_err,
                "max_speed": MAX_SPEED,
                "max_wz": MAX_WZ,
                "accel_vx": ACCEL_VX,
                "decel_vx": DECEL_VX,
                "soft_estop": self.soft_estop,
                "motor_enable": self.motor_enable,
                "cmd_age_s": None
                if not self.last_cmd_t
                else round(time.time() - self.last_cmd_t, 3),
            }

    def touch(self) -> None:
        self.last_cmd_t = time.time()

    def _hard_zero_locked(self) -> None:
        self.vx = self.vy = self.wz = 0.0
        self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
        self.coasting = False
        for k in self.wheel_vel:
            self.wheel_vel[k] = 0.0

    def set_idle(self) -> None:
        """Hard idle: zero speed immediately (process shutdown / e-stop style)."""
        with self.lock:
            self.mode = "idle"
            self._hard_zero_locked()
            self.touch()
        try:
            self._ensure_sock()
            for i in range(5):
                self._send(
                    TX_CTRL,
                    _pack_ctrl(6, 0, 0, 0, i % 16),
                )
        except OSError:
            pass

    def _latch_steer_deg(self) -> Dict[str, float]:
        """Current measured wheel angles (deg), CAN sign — for hold / free TX."""
        with self.state.lock:
            return {
                k: _clamp(math.degrees(v), -90.0, 90.0)
                for k, v in self.state.steer.items()
            }

    ZERO_STEER = {"lf": 0.0, "rf": 0.0, "lr": 0.0, "rr": 0.0}

    def _enter_hold_locked(self, ang: Optional[Dict[str, float]] = None) -> None:
        """No-command park: silent bus (do not auto-TX CAN / enable / straighten)."""
        _ = ang
        self.mode = "idle"
        self.vx = self.vy = self.wz = 0.0
        self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
        self.coasting = False
        self.gear = 6
        self.motor_enable = False
        for k in self.wheel_vel:
            self.wheel_vel[k] = 0.0
            self.wheel_angle[k] = 0.0

    def set_stop_hold(self) -> Dict[str, Any]:
        """Stop: decelerate vx/vy, snap wz→0 (回直), then hold steer=0° forever until next cmd."""
        with self.lock:
            self.wz = 0.0
            self.cmd_wz = 0.0
            for k in self.wheel_angle:
                self.wheel_angle[k] = 0.0
            nearly_stopped = (
                abs(self.cmd_vx) < VEL_EPS
                and abs(self.vx) < VEL_EPS
                and abs(self.cmd_vy) < VEL_EPS
                and abs(self.vy) < VEL_EPS
            )
            if nearly_stopped:
                self._enter_hold_locked()
                self.touch()
            else:
                self.mode = "drive"
                self.vx = 0.0
                self.vy = 0.0
                self.coasting = True
                self.motor_enable = True
                self.touch()
        return {"ok": True, "cmd": self.snapshot()}

    def set_drive(self, vx: float, wz: float, vy: float = 0.0, gear: int = 6) -> Dict[str, Any]:
        with self.lock:
            if self.soft_estop:
                blocked = True
            else:
                blocked = False
                self.mode = "drive"
                self.coasting = False
                self.gear = int(gear) & 0xFF
                self.vx = _clamp(float(vx), -MAX_SPEED, MAX_SPEED)
                self.vy = _clamp(float(vy), -MAX_SPEED, MAX_SPEED)
                self.wz = _clamp(float(wz), -MAX_WZ, MAX_WZ)
                # 前进/后退（无转向）：立刻清零角速度与转角指令，四轮保持正直
                if abs(self.wz) < WZ_EPS:
                    self.wz = 0.0
                    self.cmd_wz = 0.0
                for k in self.wheel_vel:
                    self.wheel_vel[k] = 0.0
                    self.wheel_angle[k] = 0.0
                # 运动前确保目标为使能（心跳里会重发）
                self.motor_enable = True
                self.touch()
        if blocked:
            return {"ok": False, "error": "软件急停中，拒绝运动", "cmd": self.snapshot()}
        return {"ok": True, "cmd": self.snapshot()}

    def set_free(
        self,
        wheel_vel: Optional[Dict[str, float]] = None,
        wheel_angle: Optional[Dict[str, float]] = None,
        gear: int = 6,
    ) -> Dict[str, Any]:
        with self.lock:
            if self.soft_estop:
                blocked = True
            else:
                blocked = False
                self.mode = "free"
                self.gear = int(gear) & 0xFF
                if wheel_vel:
                    for k, v in wheel_vel.items():
                        if k in self.wheel_vel:
                            self.wheel_vel[k] = _clamp(float(v), -MAX_SPEED, MAX_SPEED)
                if wheel_angle:
                    for k, v in wheel_angle.items():
                        if k in self.wheel_angle:
                            self.wheel_angle[k] = _clamp(float(v), -90.0, 90.0)
                self.touch()
        if blocked:
            return {"ok": False, "error": "软件急停中，拒绝运动", "cmd": self.snapshot()}
        return {"ok": True, "cmd": self.snapshot()}

    def _send(self, can_id: int, data: bytes) -> None:
        if self.sock is None:
            raise OSError("CAN TX socket not ready")
        frame = struct.pack(CAN_FMT, can_id | CAN_EFF_FLAG, 8, data)
        self.sock.send(frame)

    def trigger_estop(self) -> Dict[str, Any]:
        """Software e-stop: hard zero + gear disable + motor disable + brake lamp.

        Note: chassis physical e-stop button is hardware-only (io_fb). This is the
        strongest CAN-side stop available from the host.
        """
        with self.lock:
            self.soft_estop = True
            self.motor_enable = False
            self.mode = "estop"
            self.gear = 0  # disable
            self._hard_zero_locked()
            for k in self.wheel_angle:
                self.wheel_angle[k] = 0.0
            self.touch()
        try:
            self._ensure_sock()
            # 立刻连发：零速 ctrl + 电机失能 + 刹车灯
            for i in range(10):
                self._send(TX_CTRL, _pack_ctrl(0, 0, 0, 0, i % 16))
                self._send(TX_MOTOR, _pack_motor_enable(False, i % 16))
                self._send(TX_IO, _pack_io_brake(i % 16, brake=True))
                # 四轮速度也清零（覆盖 free/hold 残留）
                zeros = {"lf": 0.0, "rf": 0.0, "lr": 0.0, "rr": 0.0}
                self._send_free_locked_payload(0, zeros, zeros, i % 16)
                time.sleep(0.01)
            with self.lock:
                self._motor_hb_t = time.time()
                self._estop_hb_t = time.time()
        except OSError as exc:
            return {"ok": False, "error": str(exc), "cmd": self.snapshot()}
        return {"ok": True, "msg": "软件急停已触发：零速+档位禁用+电机失能", "cmd": self.snapshot()}

    def clear_estop(self) -> Dict[str, Any]:
        """Clear software e-stop latch; return to park-straight (0°)."""
        with self.lock:
            self.soft_estop = False
            self._enter_hold_locked()
            self.touch()
        try:
            self._ensure_sock()
            for i in range(3):
                self._send(TX_IO, _pack_io_brake(i % 16, brake=False))
                time.sleep(0.01)
        except OSError:
            pass
        return {
            "ok": True,
            "msg": "软件急停已解除，总线静默（无指令不发CAN）；如需运动请再使能并下发指令",
            "cmd": self.snapshot(),
        }

    def send_motor_enable(self, enable: bool = True) -> None:
        with self.lock:
            self.motor_enable = bool(enable)
            self._motor_cnt = (self._motor_cnt + 1) % 16
            cnt = self._motor_cnt
        self._ensure_sock()
        # 连发几帧，提高转向使能成功率
        for i in range(5):
            self._send(TX_MOTOR, _pack_motor_enable(enable, (cnt + i) % 16))
            time.sleep(0.01)
        with self.lock:
            self._motor_cnt = (cnt + 5) % 16
            self._motor_hb_t = time.time()

    def send_unlock(self) -> None:
        """安全停车解锁：手册要求 0x02→0x00 下降沿，约 20ms 间隔连发。"""
        with self.lock:
            base = self._io_cnt
        self._ensure_sock()
        seq = (0x02, 0x02, 0x00, 0x00)
        for i, val in enumerate(seq):
            d = bytearray(8)
            d[0] = val
            d[6] = ((base + i) & 0x0F) << 4
            d[7] = d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]
            self._send(TX_IO, bytes(d))
            time.sleep(0.02)
        with self.lock:
            self._io_cnt = (base + len(seq)) % 16

    def _ensure_sock(self) -> None:
        if self.sock is not None:
            return
        sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((self.can_iface,))
        self.sock = sock

    def _send_free_locked_payload(
        self,
        gear: int,
        wvel: Dict[str, float],
        wang: Dict[str, float],
        cnt: int,
    ) -> None:
        self._send(
            TX_FRONT_VEL,
            _pack_gear_s16_s16(
                gear,
                _to_s16(wvel["lf"] * 1000),
                _to_s16(wvel["rf"] * 1000),
                cnt,
            ),
        )
        self._send(
            TX_REAR_VEL,
            _pack_gear_s16_s16(
                gear,
                _to_s16(wvel["lr"] * 1000),
                _to_s16(wvel["rr"] * 1000),
                cnt,
            ),
        )
        self._send(
            TX_FRONT_ANGLE,
            _pack_gear_s16_s16(
                gear,
                _to_s16(wang["lf"] * 100),
                _to_s16(wang["rf"] * 100),
                cnt,
            ),
        )
        self._send(
            TX_REAR_ANGLE,
            _pack_gear_s16_s16(
                gear,
                _to_s16(wang["lr"] * 100),
                _to_s16(wang["rr"] * 100),
                cnt,
            ),
        )

    def _loop(self) -> None:
        period = 1.0 / CMD_HZ
        while self.running:
            t0 = time.time()
            try:
                self._ensure_sock()
                with self.lock:
                    ramp_dt = t0 - self._last_ramp_t
                    if ramp_dt < 0.0 or ramp_dt > 0.1:
                        ramp_dt = period
                    self._last_ramp_t = t0

                    age = t0 - self.last_cmd_t if self.last_cmd_t else 999.0
                    # Deadman → smooth coast (exempt while coasting)
                    if (
                        self.mode in ("drive", "free")
                        and not self.coasting
                        and age > CMD_TIMEOUT
                    ):
                        if self.mode == "free":
                            for k in self.wheel_vel:
                                self.wheel_vel[k] = 0.0
                            self._enter_hold_locked()
                        else:
                            self.vx = self.vy = self.wz = 0.0
                            self.cmd_wz = 0.0
                            for k in self.wheel_angle:
                                self.wheel_angle[k] = 0.0
                            self.coasting = True
                            self.motor_enable = True
                        self.last_cmd_t = t0

                    if self.mode == "drive":
                        self.cmd_vx = _ramp(
                            self.cmd_vx, self.vx, ACCEL_VX, DECEL_VX, ramp_dt
                        )
                        self.cmd_vy = _ramp(
                            self.cmd_vy, self.vy, ACCEL_VX, DECEL_VX, ramp_dt
                        )
                        # 停车回直：角速度强制 0；直行(无横移)时也清零 vy
                        if self.coasting:
                            self.wz = 0.0
                            self.cmd_wz = 0.0
                        elif abs(self.wz) < WZ_EPS and abs(self.vy) < VEL_EPS:
                            self.wz = 0.0
                            self.cmd_wz = 0.0
                            self.vy = 0.0
                            self.cmd_vy = 0.0
                        else:
                            self.cmd_wz = _ramp(
                                self.cmd_wz, self.wz, ACCEL_WZ, DECEL_WZ, ramp_dt
                            )
                        if (
                            self.coasting
                            and abs(self.cmd_vx) < VEL_EPS
                            and abs(self.cmd_vy) < VEL_EPS
                        ):
                            self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
                            self._enter_hold_locked()
                        elif self.coasting:
                            self.last_cmd_t = t0

                    # 无运动指令：内部清零，保持静默（不自动使能）
                    if self.mode in ("hold", "idle"):
                        for k in self.wheel_angle:
                            self.wheel_angle[k] = 0.0
                        for k in self.wheel_vel:
                            self.wheel_vel[k] = 0.0
                        self.motor_enable = False

                    mode = self.mode
                    gear = self.gear
                    vx, vy, wz = self.cmd_vx, self.cmd_vy, self.cmd_wz
                    wvel = dict(self.wheel_vel)
                    wang = dict(self.wheel_angle)
                    self._cnt = (self._cnt + 1) % 16
                    cnt = self._cnt

                if mode == "idle" or mode == "hold":
                    # 无指令：禁止自动发 CAN（仅接收反馈）
                    pass
                elif mode == "estop":
                    # 持续压制：零速 + 档位禁用 + 失能 + 刹车灯
                    self._send(TX_CTRL, _pack_ctrl(0, 0, 0, 0, cnt))
                    zeros = dict(self.ZERO_STEER)
                    self._send_free_locked_payload(0, zeros, zeros, cnt)
                    with self.lock:
                        hb = (t0 - self._estop_hb_t) >= 0.1
                        if hb:
                            self._motor_cnt = (self._motor_cnt + 1) % 16
                            self._io_cnt = (self._io_cnt + 1) % 16
                            mcnt = self._motor_cnt
                            icnt = self._io_cnt
                            self._estop_hb_t = t0
                            self._motor_hb_t = t0
                        else:
                            mcnt = icnt = None
                    if mcnt is not None:
                        self._send(TX_MOTOR, _pack_motor_enable(False, mcnt))
                        self._send(TX_IO, _pack_io_brake(icnt, brake=True))
                elif mode == "drive":
                    data = _pack_ctrl(
                        gear,
                        _to_s16(vx * 1000),
                        _to_s16(wz * 100),
                        _to_s16(vy * 1000),
                        cnt,
                    )
                    self._send(TX_CTRL, data)
                    # 运动时只发 ctrl_cmd。若同时发自由转角帧，底盘会切到单轮自由控
                    # 制而忽略协同线速度（表现为前进/后退“没反应”）。
                elif mode == "free":
                    self._send_free_locked_payload(gear, wvel, wang, cnt)

                # 周期重发使能：仅 drive/free 有指令时；idle/hold 静默
                with self.lock:
                    want_en = self.motor_enable and mode in ("drive", "free")
                    hb_due = (t0 - self._motor_hb_t) >= 0.2
                    if want_en and hb_due:
                        self._motor_cnt = (self._motor_cnt + 1) % 16
                        mcnt = self._motor_cnt
                        self._motor_hb_t = t0
                    else:
                        mcnt = None
                if mcnt is not None:
                    self._send(TX_MOTOR, _pack_motor_enable(True, mcnt))

                self.tx_ok = True
                self.tx_err = ""
            except OSError as exc:
                self.tx_ok = False
                self.tx_err = str(exc)
                if self.sock is not None:
                    try:
                        self.sock.close()
                    except OSError:
                        pass
                    self.sock = None
                time.sleep(1.0)
            elapsed = time.time() - t0
            time.sleep(max(0.0, period - elapsed))


class ChassisState:
    def __init__(self, can_iface: str = CAN_IFACE) -> None:
        self.can_iface = can_iface
        self.lock = threading.Lock()
        self.running = True
        self.can_ok = False
        self.last_rx = 0.0
        self.msg_hz = 0.0
        self._msg_count = 0
        self._hz_t0 = time.time()

        self.gear = 0
        self.vx = 0.0
        self.vy = 0.0
        self.wz_deg = 0.0
        self.steer = {"lf": 0.0, "rf": 0.0, "lr": 0.0, "rr": 0.0}
        self.wheel_spin = {"lf": 0.0, "rf": 0.0, "lr": 0.0, "rr": 0.0}
        self.wheel_vel = {"lf": 0.0, "rf": 0.0, "lr": 0.0, "rr": 0.0}
        self.angle_hz = 0.0
        self._angle_count = 0
        self._angle_hz_t0 = time.time()
        self.last_angle_rx = 0.0
        self.angle_seq = 0
        self.angle_crc_ok = True
        self.steer_cmd_vel = 0.0
        self.steer_cmd_angle = 0.0
        self.estop = False
        self.soc: Optional[int] = None
        self.voltage: Optional[float] = None
        self.current: Optional[float] = None
        self.remaining_ah: Optional[float] = None
        self.error_level = 0
        self.io = {
            "unlock": False,
            "low_beam": False,
            "brake_lamp": False,
            "charge": False,
            "joypad_online": False,
            "joypad_ctrl": False,
            "impact": False,
            "drop": False,
        }
        self.motor = {
            "drive": {"lf": False, "lr": False, "rf": False, "rr": False},
            "steer": {"lf": False, "lr": False, "rf": False, "rr": False},
        }
        self.drive_current = {"lf": 0.0, "lr": 0.0, "rf": 0.0, "rr": 0.0}
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._last_int = time.time()

    def start(self) -> None:
        threading.Thread(target=self._can_loop, name="can-rx", daemon=True).start()
        threading.Thread(target=self._integrate_loop, name="odom", daemon=True).start()

    def stop(self) -> None:
        self.running = False

    def _can_loop(self) -> None:
        # Do not count our own TX loopback as chassis feedback
        SOL_CAN_RAW = 101
        CAN_RAW_RECV_OWN_MSGS = 4
        while self.running:
            try:
                sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                try:
                    sock.setsockopt(
                        SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, struct.pack("i", 0)
                    )
                except OSError:
                    pass
                sock.bind((self.can_iface,))
                sock.settimeout(0.5)
                self.can_ok = False
                while self.running:
                    try:
                        frame = sock.recv(16)
                    except socket.timeout:
                        # 超过 1.5s 无底盘反馈 → 离线（避免回环假在线）
                        if self.last_rx and (time.time() - self.last_rx) > 1.5:
                            self.can_ok = False
                            self.msg_hz = 0.0
                        elif not self.last_rx:
                            self.can_ok = False
                            self.msg_hz = 0.0
                        continue
                    if len(frame) < 16:
                        continue
                    can_id, dlc, data = struct.unpack(CAN_FMT, frame)
                    cid = can_id & 0x1FFFFFFF
                    # 只把底盘反馈 (*EF) 算作在线心跳
                    is_fb = (cid & 0xFF) == 0xEF
                    if is_fb:
                        self._handle(cid, data[:dlc])
                        self._msg_count += 1
                        now = time.time()
                        if now - self._hz_t0 >= 1.0:
                            self.msg_hz = self._msg_count / (now - self._hz_t0)
                            self._msg_count = 0
                            self._hz_t0 = now
                        self.last_rx = now
                        self.can_ok = True
            except OSError as exc:
                self.can_ok = False
                print(f"[can] {self.can_iface}: {exc}; retry", flush=True)
                time.sleep(2.0)

    def _note_angle(self) -> None:
        now = time.time()
        self.last_angle_rx = now
        self.angle_seq += 1
        self._angle_count += 1
        if now - self._angle_hz_t0 >= 1.0:
            self.angle_hz = self._angle_count / (now - self._angle_hz_t0)
            self._angle_count = 0
            self._angle_hz_t0 = now

    def _handle(self, cid: int, d: bytes) -> None:
        if len(d) < 8:
            return
        with self.lock:
            if cid == 0x18C4D1EF:
                self.gear = d[0] & 0x0F
                self.vx = _s16_nibble(d[0], d[1], d[2]) / 1000.0
                self.wz_deg = _s16_nibble(d[2], d[3], d[4]) / 100.0
                self.vy = _s16_nibble(d[4], d[5], d[6]) / 1000.0
            elif cid == 0x18C4D2EF:
                self.steer_cmd_vel = _s16_nibble(d[0], d[1], d[2]) / 1000.0
                self.steer_cmd_angle = _s16_nibble(d[2], d[3], d[4]) / 100.0
            elif cid == 0x18C4D6EF:
                self.wheel_vel["lf"] = _s16(d[0], d[1]) / 1000.0
            elif cid == 0x18C4D7EF:
                self.wheel_vel["lr"] = _s16(d[0], d[1]) / 1000.0
            elif cid == 0x18C4D8EF:
                self.wheel_vel["rr"] = _s16(d[0], d[1]) / 1000.0
            elif cid == 0x18C4D9EF:
                self.wheel_vel["rf"] = _s16(d[0], d[1]) / 1000.0
            elif cid == 0x18C4DCEF:
                ok = (d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]) == d[7]
                self.angle_crc_ok = ok
                if ok:
                    self.steer["lf"] = math.radians(_s16(d[0], d[1]) / 100.0)
                    self.steer["rf"] = math.radians(_s16(d[2], d[3]) / 100.0)
                    self._note_angle()
            elif cid == 0x18C4DDEF:
                ok = (d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]) == d[7]
                self.angle_crc_ok = ok
                if ok:
                    self.steer["lr"] = math.radians(_s16(d[0], d[1]) / 100.0)
                    self.steer["rr"] = math.radians(_s16(d[2], d[3]) / 100.0)
                    self._note_angle()
            elif cid == 0x18C4DAEF:
                self.estop = bool(d[5] & 0x01)
                self.io = {
                    "unlock": bool(d[0] & 0x02),
                    "low_beam": bool(d[1] & 0x01),
                    "brake_lamp": bool(d[1] & 0x10),
                    "charge": bool(d[5] & 0x04),
                    "joypad_online": bool(d[5] & 0x20),
                    "joypad_ctrl": bool(d[5] & 0x02),
                    "impact": bool(d[3] & 0x3F),
                    "drop": bool(d[4] & 0x3F),
                }
            elif cid == 0x18C4DBEF:
                self.motor = {
                    "drive": {
                        "lf": bool(d[0] & 0x01),
                        "lr": bool(d[0] & 0x02),
                        "rf": bool(d[0] & 0x04),
                        "rr": bool(d[0] & 0x08),
                    },
                    "steer": {
                        "lf": bool(d[0] & 0x10),
                        "lr": bool(d[0] & 0x20),
                        "rf": bool(d[0] & 0x40),
                        "rr": bool(d[0] & 0x80),
                    },
                }
            elif cid == 0x18C4E1EF:
                self.voltage = _u16(d[0], d[1]) / 100.0
                self.current = _s16(d[2], d[3]) / 100.0
                self.remaining_ah = _u16(d[4], d[5]) / 100.0
            elif cid == 0x18C4E2EF:
                self.soc = int(d[0])
            elif cid == 0x18C4E3EF:

                def n12a(a: int, b: int) -> float:
                    v = ((b & 0x0F) << 8) | a
                    if v & 0x800:
                        v -= 0x1000
                    return v / 10.0

                def n12b(b: int, c: int) -> float:
                    v = (c << 4) | (b >> 4)
                    if v & 0x800:
                        v -= 0x1000
                    return v / 10.0

                self.drive_current = {
                    "lf": n12a(d[0], d[1]),
                    "lr": n12b(d[1], d[2]),
                    "rf": n12a(d[3], d[4]),
                    "rr": n12b(d[4], d[5]),
                }
            elif cid == 0x18C4EAEF:
                self.error_level = int(d[0])

    def _integrate_loop(self) -> None:
        r = 0.12
        while self.running:
            t0 = time.time()
            with self.lock:
                now = time.time()
                dt = max(0.0, min(0.05, now - self._last_int))
                self._last_int = now
                yaw_rate = math.radians(self.wz_deg)
                c, s = math.cos(self.yaw), math.sin(self.yaw)
                self.x += (c * self.vx - s * self.vy) * dt
                self.y += (s * self.vx + c * self.vy) * dt
                self.yaw += yaw_rate * dt
                for k, v in self.wheel_vel.items():
                    self.wheel_spin[k] = (self.wheel_spin[k] + (v / r) * dt) % (2 * math.pi)
            time.sleep(max(0.0, 0.02 - (time.time() - t0)))

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            age = time.time() - self.last_rx if self.last_rx else None
            angle_age = time.time() - self.last_angle_rx if self.last_angle_rx else None
            err_name = {0: "无故障", 0xE1: "一般", 0xE2: "严重", 0xE3: "致命"}.get(
                self.error_level, str(self.error_level)
            )
            joints = {}
            # CAN feedback sign is opposite URDF +Z (CCW+): without flip, twin shows
            # toe-out while the real chassis is toe-in (内八). HUD still shows raw CAN°.
            for k, name in STEER_JOINTS.items():
                joints[name] = -self.steer[k]
            for k, name in WHEEL_JOINTS.items():
                joints[name] = self.wheel_spin[k]
            data = {
                "t": time.time(),
                "backend": "gazebo_urdf",
                "urdf": "/assets/urdf/fw_mid.urdf",
                "can_ok": self.can_ok,
                "can_iface": self.can_iface,
                "msg_hz": round(self.msg_hz, 1),
                "data_age_s": None if age is None else round(age, 3),
                "angle_hz": round(self.angle_hz, 1),
                "angle_age_s": None if angle_age is None else round(angle_age, 3),
                "angle_seq": self.angle_seq,
                "angle_crc_ok": self.angle_crc_ok,
                "gear": self.gear,
                "gear_name": GEAR_NAME.get(self.gear, str(self.gear)),
                "vx": self.vx,
                "vy": self.vy,
                "wz_deg": self.wz_deg,
                "steer_virtual_vel": self.steer_cmd_vel,
                "steer_virtual_angle": self.steer_cmd_angle,
                "estop": self.estop,
                "soc": self.soc,
                "voltage": self.voltage,
                "current": self.current,
                "remaining_ah": self.remaining_ah,
                "error_level": self.error_level,
                "error_name": err_name,
                "io": dict(self.io),
                "motor": {
                    "drive": dict(self.motor["drive"]),
                    "steer": dict(self.motor["steer"]),
                },
                "drive_current": dict(self.drive_current),
                "pose": {"x": self.x, "y": self.y, "yaw": self.yaw},
                "steer_deg": {k: round(math.degrees(v), 2) for k, v in self.steer.items()},
                "wheel_vel": {k: round(v, 4) for k, v in self.wheel_vel.items()},
                "joints": joints,
            }
        # 勿在持有 state.lock 时再抢 commander.lock（会死锁卡住 /api/state）
        data["cmd"] = commander.snapshot() if commander else None
        return data


app = Flask(__name__, static_folder=str(STATIC), static_url_path="")
twin = ChassisState()
commander = CanCommander(CAN_IFACE, twin)


@app.route("/")
def index():
    return send_from_directory(STATIC, "index.html")


@app.route("/api/state")
def api_state():
    return jsonify(twin.snapshot())


@app.route("/api/cmd", methods=["POST"])
def api_cmd():
    body = request.get_json(force=True, silent=True) or {}
    action = str(body.get("action", "")).lower()

    # 软件急停优先：立刻执行，不走平滑停车
    if action in ("estop", "e_stop", "emergency_stop"):
        return jsonify(commander.trigger_estop())
    if action in ("estop_clear", "clear_estop"):
        return jsonify(commander.clear_estop())

    blocked = twin.estop or commander.soft_estop
    if blocked and action not in (
        "stop",
        "idle",
        "hold",
        "status",
        "enable",
        "disable",
        "unlock",
        "estop",
        "estop_clear",
        "clear_estop",
    ):
        why = "硬件急停按下" if twin.estop else "软件急停已锁存"
        return jsonify({"ok": False, "error": f"{why}，拒绝运动指令（先解除急停）"}), 400

    if action in ("stop", "idle", "hold"):
        # 普通停车：减速保持转角；若已急停则保持压制
        if commander.soft_estop:
            return jsonify(commander.trigger_estop())
        return jsonify(commander.set_stop_hold())

    if action == "enable":
        if commander.soft_estop:
            return jsonify({"ok": False, "error": "软件急停中，请先点「解除急停」"}), 400
        try:
            commander.send_motor_enable(True)
            online = bool(twin.can_ok) and twin.last_rx and (time.time() - twin.last_rx) < 1.5
            if not online:
                return jsonify(
                    {
                        "ok": False,
                        "error": "使能帧已发送，但底盘无CAN反馈（离线）。请检查：车端电源、急停旋钮、CAN线、can0。页面原先可能误报在线。",
                        "cmd": commander.snapshot(),
                    }
                ), 503
            return jsonify({"ok": True, "msg": "电机使能已发送（驱动+转向）"})
        except OSError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 500

    if action == "disable":
        try:
            commander.send_motor_enable(False)
            return jsonify({"ok": True, "msg": "电机失能已发送"})
        except OSError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 500

    if action == "straighten":
        try:
            commander.send_motor_enable(True)
        except OSError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 500
        return jsonify(
            commander.set_free(
                {"lf": 0.0, "rf": 0.0, "lr": 0.0, "rr": 0.0},
                {"lf": 0.0, "rf": 0.0, "lr": 0.0, "rr": 0.0},
                int(body.get("gear", 6)),
            )
        )

    if action == "unlock":
        try:
            commander.send_unlock()
            return jsonify({"ok": True, "msg": "解锁脉冲已发送"})
        except OSError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 500

    if action == "status":
        return jsonify({"ok": True, "cmd": commander.snapshot()})

    speed = _clamp(float(body.get("speed", 0.1)), 0.0, MAX_SPEED)
    turn = _clamp(float(body.get("turn", 25.0)), 0.0, MAX_WZ)
    gear = int(body.get("gear", 6))

    if action == "forward":
        # wz 强制 0 → 四轮保持正直前进
        return jsonify(commander.set_drive(speed, 0.0, 0.0, gear))
    if action == "backward":
        return jsonify(commander.set_drive(-speed, 0.0, 0.0, gear))
    if action in ("left", "arc_left"):
        # 前进左转
        return jsonify(commander.set_drive(speed, turn, 0.0, gear))
    if action in ("right", "arc_right"):
        return jsonify(commander.set_drive(speed, -turn, 0.0, gear))
    if action in ("back_left", "arc_back_left"):
        # 后退左转：负线速度 + 同向角速度
        return jsonify(commander.set_drive(-speed, turn, 0.0, gear))
    if action in ("back_right", "arc_back_right"):
        return jsonify(commander.set_drive(-speed, -turn, 0.0, gear))
    if action in ("strafe_left", "shift_left", "left_shift"):
        # 横移档(8)：+Y 为向左平移（ISO/ROS）
        return jsonify(commander.set_drive(0.0, 0.0, speed, 8))
    if action in ("strafe_right", "shift_right", "right_shift"):
        return jsonify(commander.set_drive(0.0, 0.0, -speed, 8))
    if action == "drive":
        vx = float(body.get("vx", 0.0))
        wz = float(body.get("wz", 0.0))
        vy = float(body.get("vy", 0.0))
        return jsonify(commander.set_drive(vx, wz, vy, gear))
    if action == "hold":
        # deadman refresh while holding a drive button
        commander.touch()
        if commander.mode == "idle":
            return jsonify({"ok": True, "cmd": commander.snapshot(), "note": "idle"})
        return jsonify({"ok": True, "cmd": commander.snapshot()})
    if action == "wheel":
        # 单轮：指定轮的速度/角度，其余保持当前 free 设定
        wheel = str(body.get("wheel", "")).lower()
        if wheel not in ("lf", "rf", "lr", "rr"):
            return jsonify({"ok": False, "error": "wheel 必须是 lf/rf/lr/rr"}), 400
        vel = body.get("velocity", body.get("v", 0.0))
        ang = body.get("angle", body.get("a", 0.0))
        wvel = {wheel: float(vel)}
        wang = {wheel: float(ang)}
        # zero other wheels' velocity for safety unless keep_others
        if not body.get("keep_others"):
            for k in ("lf", "rf", "lr", "rr"):
                if k != wheel:
                    wvel[k] = 0.0
        return jsonify(commander.set_free(wvel, wang, gear))
    if action == "free":
        return jsonify(
            commander.set_free(
                body.get("wheel_vel"),
                body.get("wheel_angle"),
                gear,
            )
        )

    return jsonify({"ok": False, "error": f"未知 action: {action}"}), 400


@app.route("/assets/<path:filename>")
def assets(filename: str):
    return send_from_directory(ASSETS, filename)


def main() -> None:
    print(f"[web] Gazebo URDF twin  CAN={CAN_IFACE}  http://{HOST}:{PORT}/", flush=True)
    twin.start()
    commander.start()
    app.run(host=HOST, port=PORT, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
