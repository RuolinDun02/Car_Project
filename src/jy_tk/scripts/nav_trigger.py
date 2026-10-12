#!/usr/bin/env python3
"""
导航触发状态机节点（话题发送目标版 + 等清完地图再发目标点）

流程：
  IDLE → BACKUP → SPIN_360 → WAIT → HANDOFF → CLEAR_MAP → SEND_GOAL → NAVIGATING → DONE/FAILED

  HANDOFF：只负责"切到 CLEAR_MAP"
  CLEAR_MAP：发清地图请求，非阻塞地检查完成；完成后切到 SEND_GOAL
  SEND_GOAL：发目标点，切到 NAVIGATING

★ 清地图是"真等待"——必须等到服务返回或超时，才继续发目标点
★ 全程不嵌套 spin，不崩溃
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time as RclpyTime
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatusArray
from nav2_msgs.srv import ClearEntireCostmap

import tf2_ros
from tf2_ros import TransformException


class State(Enum):
    IDLE        = "IDLE"
    BACKUP      = "BACKUP"
    SPIN_360    = "SPIN_360"
    WAIT        = "WAIT"
    HANDOFF     = "HANDOFF"
    CLEAR_MAP   = "CLEAR_MAP"      # ★ 新增：清地图阶段
    SEND_GOAL   = "SEND_GOAL"      # ★ 新增：发目标点阶段
    NAVIGATING  = "NAVIGATING"
    DONE        = "DONE"
    FAILED      = "FAILED"


class NavTrigger(Node):
    def __init__(self):
        super().__init__('nav_trigger')

        # ---------- 参数 ----------
        self.declare_parameter('goal_x',   2.0)
        self.declare_parameter('goal_y',   1.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('backup_dist', 1.0)
        self.declare_parameter('backup_speed', 0.15)
        self.declare_parameter('backup_timeout', 10.0)
        self.declare_parameter('spin_360_wz_deg', 60.0)
        self.declare_parameter('spin_360_timeout', 60.0)
        self.declare_parameter('wait_after_spin', 2.0)
        self.declare_parameter('clear_map_timeout', 3.0)   # ★ 清地图超时（秒）
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_smoothed')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('nav_status_topic',
                               '/navigate_to_pose/_action/status')
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
        self.wait_after_spin = p('wait_after_spin').value
        self.clear_map_timeout = p('clear_map_timeout').value
        self.control_rate     = p('control_rate').value
        self.cmd_vel_topic    = p('cmd_vel_topic').value
        self.goal_pose_topic  = p('goal_pose_topic').value
        self.nav_status_topic = p('nav_status_topic').value
        self.map_frame        = p('map_frame').value
        self.base_frame       = p('base_frame').value

        # ---------- 目标点 ----------
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
        self.backup_start_xy  = (0.0, 0.0)
        self._last_yaw        = 0.0
        self._accumulated_yaw = 0.0

        # ★ 清地图阶段的 future 句柄
        self._clear_global_future = None
        self._clear_local_future  = None

        # ---------- TF ----------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- 清地图客户端 ----------
        self.clear_global_cli = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )
        self.clear_local_cli = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
        )

        # ---------- 发布 ----------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(String, '/nav_trigger/state', state_qos)
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, self.goal_pose_topic, 10)

        # ---------- 订阅 ----------
        self.create_subscription(Bool, '/nav_trigger', self.on_trigger, 10)
        nav_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            GoalStatusArray,
            self.nav_status_topic,
            self.on_nav_status,
            nav_status_qos,
        )

        self.create_timer(1.0 / self.control_rate, self.tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('nav_trigger 已启动')
        self.get_logger().info(
            f'  目标点: ({self.goal_x:.2f}, {self.goal_y:.2f})')
        self.get_logger().info(
            f'  BACKUP: {self.backup_dist}m @ {self.backup_speed}m/s')
        self.get_logger().info(
            f'  SPIN_360: {self.spin_360_wz_deg}°/s')
        self.get_logger().info(f'  WAIT: {self.wait_after_spin}s')
        self.get_logger().info(f'  清地图超时: {self.clear_map_timeout}s')
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

    def on_nav_status(self, msg: GoalStatusArray):
        if self.state != State.NAVIGATING:
            return
        if not msg.status_list:
            return
        latest = msg.status_list[-1]
        status = latest.status
        if status == 4:
            self.get_logger().info('导航成功')
            self._transition(State.DONE)
        elif status in (5, 6):
            self.get_logger().warn(f'导航结束，status={status}')
            self._transition(State.FAILED)

    # ================================================================
    # 状态切换
    # ================================================================
    def _transition(self, new_state: State):
        self.get_logger().info(f'状态: {self.state.value} → {new_state.value}')
        self.state = new_state
        self.state_enter_time = self.get_clock().now()

        # 进入非"主动控制"状态时，清一次零速度
        if new_state not in (State.BACKUP, State.SPIN_360):
            self._publish_zero()

        if new_state == State.BACKUP:
            cur = self._get_current_pose()
            if cur is not None:
                self.backup_start_xy = (cur[0], cur[1])
        elif new_state == State.SPIN_360:
            cur = self._get_current_pose()
            if cur is not None:
                self._last_yaw = cur[2]
            self._accumulated_yaw = 0.0
        elif new_state == State.CLEAR_MAP:
            # ★ 进入清地图状态：发请求
            self._start_clear_costmaps()
        elif new_state == State.SEND_GOAL:
            # ★ 进入发目标点状态：发目标
            self._publish_goal()

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
        if s == State.BACKUP:
            self._do_backup()
        elif s == State.SPIN_360:
            self._do_spin_360()
        elif s == State.WAIT:
            self._do_wait()
        elif s == State.HANDOFF:
            self._do_handoff()
        elif s == State.CLEAR_MAP:
            self._do_clear_map()
        elif s == State.SEND_GOAL:
            self._do_send_goal()
        # IDLE / DONE / FAILED / NAVIGATING 都不发任何速度

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
        self._send_cmd(vx=-self.backup_speed, vy=0.0, wz_deg=0.0)

    # ================================================================
    # 阶段 2：旋转 360°
    # ================================================================
    def _do_spin_360(self):
        if self._elapsed() > self.spin_360_timeout:
            self.get_logger().warn(
                f'旋转 360° 超时（累计 {math.degrees(self._accumulated_yaw):.1f}°）')
            self._transition(State.WAIT)
            return
        cur = self._get_current_pose()
        if cur is None:
            self.get_logger().warn('无法获取位姿，跳过旋转')
            self._transition(State.WAIT)
            return
        cur_yaw = cur[2]
        delta = self._norm_angle(cur_yaw - self._last_yaw)
        self._accumulated_yaw += delta
        self._last_yaw = cur_yaw
        if abs(self._accumulated_yaw) >= 2.0 * math.pi:
            self.get_logger().info(
                f'旋转 360° 完成 (累计 {math.degrees(self._accumulated_yaw):.1f}°)')
            self._transition(State.WAIT)
            return
        self._send_cmd(vx=0.0, vy=0.0, wz_deg=self.spin_360_wz_deg)

    # ================================================================
    # 阶段 3：等待
    # ================================================================
    def _do_wait(self):
        if self._elapsed() < self.wait_after_spin:
            return
        cur = self._get_current_pose()
        if cur is not None:
            self.get_logger().info(
                f'等待完成。当前位置: ({cur[0]:.2f}, {cur[1]:.2f}, '
                f'yaw={math.degrees(cur[2]):.1f}°)')
        else:
            self.get_logger().warn('等待完成但 TF 不可用')
        self._transition(State.HANDOFF)

    # ================================================================
    # 阶段 4：HANDOFF → 切到 CLEAR_MAP
    # ================================================================
    def _do_handoff(self):
        """只负责切到 CLEAR_MAP 状态。实际清地图在 CLEAR_MAP 里做。"""
        self._transition(State.CLEAR_MAP)

    # ================================================================
    # 阶段 5：CLEAR_MAP → 等清地图完成 → 切到 SEND_GOAL
    # ================================================================
    def _start_clear_costmaps(self):
        """进入 CLEAR_MAP 时调用：发清地图请求（异步，不等待）。"""
        req = ClearEntireCostmap.Request()

        if self.clear_global_cli.service_is_ready():
            self._clear_global_future = self.clear_global_cli.call_async(req)
        else:
            self._clear_global_future = None
            self.get_logger().warn('global_costmap 服务未就绪')

        if self.clear_local_cli.service_is_ready():
            self._clear_local_future = self.clear_local_cli.call_async(req)
        else:
            self._clear_local_future = None
            self.get_logger().warn('local_costmap 服务未就绪')

    def _do_clear_map(self):
        """CLEAR_MAP 状态：每帧检查两个清地图 future 是否完成。

        完成 → 切到 SEND_GOAL
        超时 → 也切到 SEND_GOAL（打印警告）
        """
        # 超时检查
        if self._elapsed() > self.clear_map_timeout:
            self.get_logger().warn(
                f'清地图超时 ({self.clear_map_timeout}s)，继续发目标点')
            self._transition(State.SEND_GOAL)
            return

        # 检查两个 future 是否都完成
        global_done = (self._clear_global_future is None or
                       self._clear_global_future.done())
        local_done = (self._clear_local_future is None or
                      self._clear_local_future.done())

        if global_done and local_done:
            self.get_logger().info('清地图完成')
            self._transition(State.SEND_GOAL)

    # ================================================================
    # 阶段 6：SEND_GOAL → 发目标点 → 切到 NAVIGATING
    # ================================================================
    def _publish_goal(self):
        """进入 SEND_GOAL 时调用：发目标点。"""
        msg = PoseStamped()
        msg.header.frame_id = self.goal_frame
        msg.header.stamp = RclpyTime().to_msg()
        msg.pose = self.goal.pose
        self.goal_pose_pub.publish(msg)
        self.get_logger().info(
            f'已通过话题 {self.goal_pose_topic} 发送目标点 '
            f'({self.goal_x:.2f}, {self.goal_y:.2f})')

    def _do_send_goal(self):
        """SEND_GOAL 状态：立即切到 NAVIGATING。

        （目标点已在 _publish_goal 里发了，这里只切状态）
        """
        self._transition(State.NAVIGATING)

    # ================================================================
    # 辅助
    # ================================================================
    def _send_cmd(self, vx: float, vy: float, wz_deg: float):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.angular.z = math.radians(wz_deg)
        self.cmd_pub.publish(msg)

    def _publish_zero(self):
        msg = Twist()
        msg.linear.x  = 0.0
        msg.linear.y  = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def _get_current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                RclpyTime(),
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
        try:
            node._publish_zero()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
