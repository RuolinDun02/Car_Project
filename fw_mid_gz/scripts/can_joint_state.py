#!/usr/bin/env python3
"""Publish FW-mid JointState from live chassis CAN (digital twin)."""

from __future__ import annotations

import math
import os
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

CAN_IFACE = os.environ.get("CHASSIS_CAN", "can0")
CAN_FMT = "=IB3x8s"

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


def s16(lo: int, hi: int) -> int:
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v


class CanJointState(Node):
    def __init__(self) -> None:
        super().__init__("fw_mid_can_joint_state")
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.steer = {k: 0.0 for k in STEER_JOINTS}
        self.wheel_spin = {k: 0.0 for k in WHEEL_JOINTS}
        self.wheel_vel = {k: 0.0 for k in WHEEL_JOINTS}
        self.lock = threading.Lock()
        self._last = time.time()
        self.running = True
        threading.Thread(target=self._can_loop, daemon=True).start()
        self.create_timer(0.02, self._tick)  # 50 Hz
        self.get_logger().info(f"CAN twin on {CAN_IFACE} → /joint_states")

    def _can_loop(self) -> None:
        while self.running and rclpy.ok():
            try:
                sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                sock.bind((CAN_IFACE,))
                sock.settimeout(1.0)
                while self.running and rclpy.ok():
                    try:
                        frame = sock.recv(16)
                    except socket.timeout:
                        continue
                    if len(frame) < 16:
                        continue
                    can_id, dlc, data = struct.unpack(CAN_FMT, frame)
                    self._handle(can_id & 0x1FFFFFFF, data[:dlc])
            except OSError as exc:
                self.get_logger().warn(f"{CAN_IFACE}: {exc}; retry")
                time.sleep(2.0)

    def _handle(self, cid: int, d: bytes) -> None:
        if len(d) < 8:
            return
        with self.lock:
            if cid == 0x18C4DCEF:
                if (d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]) == d[7]:
                    self.steer["lf"] = math.radians(s16(d[0], d[1]) / 100.0)
                    self.steer["rf"] = math.radians(s16(d[2], d[3]) / 100.0)
            elif cid == 0x18C4DDEF:
                if (d[0] ^ d[1] ^ d[2] ^ d[3] ^ d[4] ^ d[5] ^ d[6]) == d[7]:
                    self.steer["lr"] = math.radians(s16(d[0], d[1]) / 100.0)
                    self.steer["rr"] = math.radians(s16(d[2], d[3]) / 100.0)
            elif cid == 0x18C4D6EF:
                self.wheel_vel["lf"] = s16(d[0], d[1]) / 1000.0
            elif cid == 0x18C4D7EF:
                self.wheel_vel["lr"] = s16(d[0], d[1]) / 1000.0
            elif cid == 0x18C4D8EF:
                self.wheel_vel["rr"] = s16(d[0], d[1]) / 1000.0
            elif cid == 0x18C4D9EF:
                self.wheel_vel["rf"] = s16(d[0], d[1]) / 1000.0

    def _tick(self) -> None:
        now = time.time()
        dt = max(0.0, min(0.05, now - self._last))
        self._last = now
        r = 0.12
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            for k, v in self.wheel_vel.items():
                self.wheel_spin[k] = (self.wheel_spin[k] + (v / r) * dt) % (2 * math.pi)
            for k, jn in STEER_JOINTS.items():
                msg.name.append(jn)
                msg.position.append(self.steer[k])
            for k, jn in WHEEL_JOINTS.items():
                msg.name.append(jn)
                msg.position.append(self.wheel_spin[k])
                msg.velocity.append(self.wheel_vel[k] / r)
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CanJointState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.running = False
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
