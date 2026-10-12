#!/usr/bin/env python3
"""
导航触发状态机节点

流程：
  IDLE → BACKUP → SPIN_360 → DIAGONAL_MOVE → ALIGN → HANDOFF → NAVIGATING → DONE

  BACKUP        后退一段距离
  SPIN_360      原地旋转 360°
  DIAGONAL_MOVE 斜向平移一段距离
  ALIGN         旋转对齐到目标方向
  HANDOFF       交给 Nav2 的 navigate_to_pose action
  NAVIGATING    等待 Nav2 完成

速度指令统一发到 /cmd_vel_smoothed（Twist 类型），
由 cmd_vel_to_ctrl_cmd_bridge 节点转换成 CtrlCmd 发给底盘。
桥接节点会根据 Twist.linear.y 自动切换 gear 6/8。
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException


class State(Enum):
    IDLE          = "IDLE"
    BACKUP        = "BACKUP"
    SPIN_360      = "SPIN_360"
    DIAGONAL_MOVE = "DIAGONAL_MOVE"
    ALIGN         = "ALIGN"
    HANDOFF       = "HANDOFF"
    NAVIGATING    = "NAVIGATING"
    DONE          = "DONE"
    FAILED        = "FAILED"


class NavTrigger(Node):
    def __init__(self):
        super().__init__('nav_trigger')

        # ==========================================================
        # 目标点
        # ==========================================================
        self.declare_parameter('goal_x',   2.0)
        self.declare_parameter('goal_y',   1.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('goal_frame', 'map')

        # ---------- BACKUP ----------
        self.declare_parameter('backup_dist', 0.3)
        self.declare_parameter('backup_speed', 0.15)
        self.declare_parameter('backup_timeout', 10.0)

        # ---------- SPIN_360 ----------
        self.declare_parameter('spin_360_wz_deg', 60.0)
        self.declare_parameter('spin_360_timeout', 30.0)

        # ---------- DIAGONAL_MOVE ----------
        self.declare_parameter('diagonal_dist', 0.5)
        self.declare_parameter('diagonal_speed', 0.15)
        self.declare_parameter('diagonal_angle_deg', 45.0)
        self.declare_parameter('diagonal_timeout', 15.0)

        # ---------- ALIGN ----------
        self.declare_parameter('align_tolerance_rad', 0.10)
        self.declare_parameter('align_max_wz_deg', 30.0)
        self.declare_parameter('align_kp', 1.5)
        self.declare_parameter('align_timeout', 15.0)

        # ---------- 控制 ----------
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_smoothed')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        p = self.get_parameter
        self.goal_x     = p('goal_x').value
        self.goal_y     = p('goal_y').value
        self.goal_yaw   = p('goal_yaw').value
        self.goal_frame = p('goal_frame').value

        self.backup_dist    = p('backup_dist').value
        self.backup_speed   = p('backup_speed').value
        self.backup_timeout = p('backup_timeout').value

        self.spin_360_wz_deg  = p('spin_360_wz_deg').value
        self.spin_360_timeout = p('spin_360_timeout').value

        self.diagonal_dist      = p('diagonal_dist').value
        self.diagonal_speed     = p('diagonal_speed').value
        self.diagonal_angle_deg = p('diagonal_angle_deg').value
        self.diagonal_timeout   = p('diagonal_timeout').value

        self.align_tol     = p('align_tolerance_rad').value
        self.align_max_wz  = math.radians(p('align_max_wz_deg').value)
        self.align_kp      = p('align_kp').value
        self.align_timeout = p('align_timeout').value

        self.control_rate  = p('control_rate').value
        self.cmd_vel_topic = p('cmd_vel_topic').value
        self.map_frame     = p('map_frame').value
        self.base_frame    = p('base_frame').value

        # ---------- 构造目标 PoseStamped ----------
        self.goal = PoseStamped()
        self.goal.header.frame_id = self.goal_frame
        self.goal.pose.position.x = self.goal_x
        self.goal.pose.position.y = self.goal_y
        self.goal.pose.position.z = 0.0
        self.goal.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        self.goal.pose.orientation.w = math.cos(self.goal_yaw / 2.0)

        # ---------- 状态 ----------
        self.state            = State.IDLE
        self.state_enter_time = self.get_clock().now()

        # ---------- 里程跟踪 ----------
        self.backup_start_xy  = (0.0, 0.0)
        self.diag_start_xy    = (0.0, 0.0)
        self._last_yaw        = 0.0
        self._accumulated_yaw = 0.0

        # ---------- TF ----------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- 发布 ----------
        self.cmd_pub   = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, '/nav_trigger/state', 10)

        # ---------- 订阅 ----------
        self.create_subscription(Bool, '/nav_trigger', self.on_trigger, 10)

        # ---------- Nav2 Action Client ----------
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ---------- 定时器 ----------
        self.create_timer(1.0 / self.control_rate, self.tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('nav_trigger 已启动')
        self.get_logger().info(
            f'  目标点: ({self.goal_x:.2f}, {self.goal_y:.2f}) '
            f'yaw={math.degrees(self.goal_yaw):.1f}° frame={self.goal_frame}')
        self.get_logger().info(
            f'  BACKUP: {self.backup_dist}m @ {self.backup_speed}m/s')
        self.get_logger().info(
            f'  SPIN_360: {self.spin_360_wz_deg}°/s')
        self.get_logger().info(
            f'  DIAGONAL: {self.diagonal_dist}m @ {self.diagonal_speed}m/s '
            f'角度={self.diagonal_angle_deg}°')
        self.get_logger().info(
            f'  ALIGN: 容差 {math.degrees(self.align_tol):.1f}°')
        self.get_logger().info(f'  速度话题: {self.cmd_vel_topic}')
        self.get_logger().info('=' * 60)

        self._publish_state()

    # ================================================================
    # 回调
    # ================================================================
    def on_trigger(self, msg: Bool):
        if not msg.data:
            return
        if self.state == State.NAVIGATING:
            self.get_logger().warn('导航进行中，忽略触发')
            return

        self.get_logger().info('收到触发信号')
        self._transition(State.BACKUP)

    # ================================================================
    # 状态切换
    # ================================================================
    def _transition(self, new_state: State):
        self.get_logger().info(f'状态: {self.state.value} → {new_state.value}')
        self.state = new_state
        self.state_enter_time = self.get_clock().now()

        if new_state == State.BACKUP:
            cur = self._get_current_pose()
            if cur is not None:
                self.backup_start_xy = (cur[0], cur[1])

        elif new_state == State.SPIN_360:
            cur = self._get_current_pose()
            if cur is not None:
                self._last_yaw = cur[2]
            self._accumulated_yaw = 0.0

        elif new_state == State.DIAGONAL_MOVE:
            cur = self._get_current_pose()
            if cur is not None:
                self.diag_start_xy = (cur[0], cur[1])

        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    # ================================================================
    # 状态机主循环
    # ================================================================
    def tick(self):
        s = self.state
        if s in (State.IDLE, State.DONE, State.FAILED):
            self._publish_zero()
        elif s == State.BACKUP:
            self._do_backup()
        elif s == State.SPIN_360:
            self._do_spin_360()
        elif s == State.DIAGONAL_MOVE:
            self._do_diagonal_move()
        elif s == State.ALIGN:
            self._do_align()
        elif s == State.HANDOFF:
            self._do_handoff()
        # NAVIGATING: Nav2 控制，不发指令

    # ================================================================
    # 阶段 1：后退
    # ================================================================
    def _do_backup(self):
        if self._elapsed() > self.backup_timeout:
            self.get_logger().warn('后退超时 → 旋转')
            self._transition(State.SPIN_360)
            return

        cur = self._get_current_pose()
        if cur is None:
            self.get_logger().warn('无法获取位姿，跳过后退')
            self._transition(State.SPIN_360)
            return

        dx = cur[0] - self.backup_start_xy[0]
        dy = cur[1] - self.backup_start_xy[1]
        dist = math.hypot(dx, dy)

        if dist >= self.backup_dist:
            self.get_logger().info(f'后退完成 ({dist:.3f} m)')
            self._transition(State.SPIN_360)
            return

        # 后退：vx 负值，vy=0, wz=0
        self._send_cmd(vx=-self.backup_speed, vy=0.0, wz_deg=0.0)

    # ================================================================
    # 阶段 2：旋转 360°
    # ================================================================
    def _do_spin_360(self):
        if self._elapsed() > self.spin_360_timeout:
            self.get_logger().warn(
                f'旋转 360° 超时（累计 {math.degrees(self._accumulated_yaw):.1f}°）')
            self._transition(State.DIAGONAL_MOVE)
            return

        cur = self._get_current_pose()
        if cur is None:
            self.get_logger().warn('无法获取位姿，跳过旋转')
            self._transition(State.DIAGONAL_MOVE)
            return

        cur_yaw = cur[2]

        # 累计角度（处理 ±π 跳变）
        delta = self._norm_angle(cur_yaw - self._last_yaw)
        self._accumulated_yaw += delta
        self._last_yaw = cur_yaw

        if abs(self._accumulated_yaw) >= 2.0 * math.pi:
            self.get_logger().info(
                f'旋转 360° 完成 (累计 {math.degrees(self._accumulated_yaw):.1f}°)')
            self._transition(State.DIAGONAL_MOVE)
            return

        # 原地旋转：vy=0, vx=0, wz=±
        self._send_cmd(vx=0.0, vy=0.0, wz_deg=self.spin_360_wz_deg)

    # ================================================================
    # 阶段 3：斜向 45° 平移
    # ================================================================
    def _do_diagonal_move(self):
        if self._elapsed() > self.diagonal_timeout:
            self.get_logger().warn('斜向移动超时 → 对齐')
            self._transition(State.ALIGN)
            return

        cur = self._get_current_pose()
        if cur is None:
            self.get_logger().warn('无法获取位姿，跳过斜移')
            self._transition(State.ALIGN)
            return

        dx = cur[0] - self.diag_start_xy[0]
        dy = cur[1] - self.diag_start_xy[1]
        dist = math.hypot(dx, dy)

        if dist >= self.diagonal_dist:
            self.get_logger().info(f'斜向移动完成 ({dist:.3f} m)')
            self._transition(State.ALIGN)
            return

        # 相对车身方向分解成 vx, vy
        angle_rad = math.radians(self.diagonal_angle_deg)
        vx = self.diagonal_speed * math.cos(angle_rad)
        vy = self.diagonal_speed * math.sin(angle_rad)

        # 斜向平移：vx + vy，无旋转
        # 桥接节点会检测到 vy ≠ 0 → 自动切 gear 8
        self._send_cmd(vx=vx, vy=vy, wz_deg=0.0)

    # ================================================================
    # 阶段 4：旋转对齐到目标方向
    # ================================================================
    def _do_align(self):
        if self._elapsed() > self.align_timeout:
            self.get_logger().warn('对齐超时 → 交接')
            self._transition(State.HANDOFF)
            return

        cur = self._get_current_pose()
        if cur is None:
            self.get_logger().warn('无位姿，跳过对齐')
            self._transition(State.HANDOFF)
            return

        cur_x, cur_y, cur_yaw = cur

        # 目标方向（从当前位置指向目标点）
        target_yaw = math.atan2(self.goal_y - cur_y, self.goal_x - cur_x)
        err = self._norm_angle(target_yaw - cur_yaw)

        if abs(err) < self.align_tol:
            self.get_logger().info(f'对齐完成 (误差 {math.degrees(err):.2f}°)')
            self._transition(State.HANDOFF)
            return

        wz_rad = self._clamp(self.align_kp * err,
                             -self.align_max_wz, self.align_max_wz)
        wz_deg = math.degrees(wz_rad)

        # 原地旋转对齐：vx=0, vy=0, wz≠0
        self._send_cmd(vx=0.0, vy=0.0, wz_deg=wz_deg)

    # ================================================================
    # 阶段 5：交给 Nav2
    # ================================================================
    def _do_handoff(self):
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('navigate_to_pose 不可用')
            self._transition(State.FAILED)
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.goal
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.nav_client.send_goal_async(goal).add_done_callback(
            self._on_nav_response)

        self._transition(State.NAVIGATING)
        self.get_logger().info('已交给 Nav2')

    def _on_nav_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 拒绝目标')
            self._transition(State.FAILED)
            return
        self.get_logger().info('Nav2 接受目标')
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('导航成功')
            self._transition(State.DONE)
        else:
            self.get_logger().warn(f'导航结束，status={status}')
            self._transition(State.FAILED)

    # ================================================================
    # 辅助
    # ================================================================
    def _send_cmd(self, vx: float, vy: float, wz_deg: float):
        """发 Twist 到 /cmd_vel_smoothed。单位：vx/vy=m/s, wz=deg/s。"""
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.angular.z = math.radians(wz_deg)
        self.cmd_pub.publish(msg)

    def _publish_zero(self):
        """发零速度 Twist。"""
        msg = Twist()
        msg.linear.x  = 0.0
        msg.linear.y  = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def _get_current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except TransformException:
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        return x, y, self._quat_to_yaw(q.x, q.y, q.z, q.w)

    @staticmethod
    def _quat_to_yaw(x, y, z, w):
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    @staticmethod
    def _norm_angle(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _elapsed(self):
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9


def main():
    rclpy.init()
    node = NavTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 优雅退出：发零速度，然后关闭
        try:
            node._publish_zero()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()