#!/usr/bin/env python3
"""
对接控制器：反光条对准 + 直线回退

流程：
  1. 收到 /docking/request
  2. 尝试识别反光条（/tanway_lines）
  3. 若识别到 → 用反光条对准（PID 控制）
  4. 若识别不到/丢失/超时 → 回退直线对接（用 odom 位移）
  5. 完成 → 发 /docking/complete

接口：
  输入：
    /docking/request  String  触发对接，"start" 或点号
    /tanway_lines     LineSegment3DArray  反光条线段
  输出：
    /docking/complete String  "aligned" / "fallback" / "timeout" / "failed"
    /docking/state    String  状态反馈
    /cmd_vel_smoothed Twist   速度指令
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import tf2_ros
from tf2_ros import TransformException

try:
    from tanway_line_extractor.msg import LineSegment3DArray
    HAS_LINE_MSG = True
except ImportError:
    HAS_LINE_MSG = False
    LineSegment3DArray = None


class DockingState(Enum):
    IDLE       = "IDLE"
    WAIT_LINES = "WAIT_LINES"
    ALIGNING   = "ALIGNING"
    FALLBACK   = "FALLBACK"
    COMPLETE   = "COMPLETE"
    FAILED     = "FAILED"


class DockingController(Node):
    def __init__(self):
        super().__init__("docking_controller")

        # -------- 话题 --------
        self.declare_parameter("lines_topic", "/tanway_lines")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_smoothed")
        self.declare_parameter("request_topic", "/docking/request")
        self.declare_parameter("complete_topic", "/docking/complete")
        self.declare_parameter("state_topic", "/docking/state")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        # -------- 反光条筛选 --------
        self.declare_parameter("min_line_length", 0.15)
        self.declare_parameter("max_line_length", 1.50)
        self.declare_parameter("min_target_x", 0.20)
        self.declare_parameter("max_target_x", 3.00)
        self.declare_parameter("max_target_y", 0.80)

        # -------- 对准目标 --------
        self.declare_parameter("target_distance", 0.25)
        self.declare_parameter("distance_tolerance", 0.05)
        self.declare_parameter("lateral_tolerance", 0.03)

        # -------- PID --------
        self.declare_parameter("kp_lateral", 1.5)
        self.declare_parameter("kp_yaw", 2.0)
        self.declare_parameter("vx_max", 0.10)
        self.declare_parameter("vx_min", 0.03)
        self.declare_parameter("wz_max", 0.40)

        # -------- 时间约束 --------
        self.declare_parameter("wait_lines_timeout", 3.0)
        self.declare_parameter("line_lost_timeout", 1.0)
        self.declare_parameter("align_overall_timeout", 30.0)

        # -------- 直线回退 --------
        self.declare_parameter("fallback_dock_speed", 0.08)
        self.declare_parameter("fallback_dock_distance", 0.7)
        self.declare_parameter("fallback_timeout", 20.0)

        self.declare_parameter("control_rate", 20.0)

        p = self.get_parameter
        self.lines_topic = p("lines_topic").value
        self.cmd_vel_topic = p("cmd_vel_topic").value
        self.request_topic = p("request_topic").value
        self.complete_topic = p("complete_topic").value
        self.state_topic = p("state_topic").value
        self.odom_frame = p("odom_frame").value
        self.base_frame = p("base_frame").value

        self.min_line_length = float(p("min_line_length").value)
        self.max_line_length = float(p("max_line_length").value)
        self.min_target_x = float(p("min_target_x").value)
        self.max_target_x = float(p("max_target_x").value)
        self.max_target_y = float(p("max_target_y").value)

        self.target_distance = float(p("target_distance").value)
        self.distance_tolerance = float(p("distance_tolerance").value)
        self.lateral_tolerance = float(p("lateral_tolerance").value)

        self.kp_lateral = float(p("kp_lateral").value)
        self.kp_yaw = float(p("kp_yaw").value)
        self.vx_max = float(p("vx_max").value)
        self.vx_min = float(p("vx_min").value)
        self.wz_max = float(p("wz_max").value)

        self.wait_lines_timeout = float(p("wait_lines_timeout").value)
        self.line_lost_timeout = float(p("line_lost_timeout").value)
        self.align_overall_timeout = float(p("align_overall_timeout").value)

        self.fallback_dock_speed = float(p("fallback_dock_speed").value)
        self.fallback_dock_distance = float(p("fallback_dock_distance").value)
        self.fallback_timeout = float(p("fallback_timeout").value)

        self.control_rate = float(p("control_rate").value)

        # -------- 状态 --------
        self.state = DockingState.IDLE
        self.state_enter_time = self.get_clock().now()

        self.latest_lines = None
        self.lines_received_time = None

        self.wait_start_time = None
        self.align_start_time = None
        self.fallback_origin_xy = None
        self.fallback_start_time = None

        # -------- TF --------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -------- 发布 --------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.complete_pub = self.create_publisher(String, self.complete_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        # -------- 订阅 --------
        self.create_subscription(String, self.request_topic, self.on_request, 10)
        if HAS_LINE_MSG:
            self.create_subscription(
                LineSegment3DArray, self.lines_topic, self.on_lines, 10)
        else:
            self.get_logger().error(
                "无法导入 tanway_line_extractor/LineSegment3DArray，"
                "反光条对接不可用，将只走直线回退")

        # -------- 定时器 --------
        self.create_timer(1.0 / self.control_rate, self.on_timer)

        self.get_logger().info("=" * 60)
        self.get_logger().info("docking_controller 已启动")
        self.get_logger().info(f"  请求: {self.request_topic}")
        self.get_logger().info(f"  完成: {self.complete_topic}")
        self.get_logger().info(f"  反光条: {self.lines_topic}")
        self.get_logger().info(f"  速度: {self.cmd_vel_topic}")
        self.get_logger().info(f"  反光条消息可用: {HAS_LINE_MSG}")
        self.get_logger().info("=" * 60)

    # ================================================================
    # 输入回调
    # ================================================================
    def on_request(self, msg: String):
        if self.state not in (DockingState.IDLE, DockingState.COMPLETE,
                              DockingState.FAILED):
            self.get_logger().warn(
                f"当前 {self.state.value}，忽略对接请求")
            return

        self.get_logger().info(f"收到对接请求: {msg.data}")
        self._publish_zero()
        self.wait_start_time = self.get_clock().now()
        self._transition(DockingState.WAIT_LINES)

    def on_lines(self, msg):
        self.latest_lines = msg
        self.lines_received_time = self.get_clock().now()

    # ================================================================
    # 状态机
    # ================================================================
    def _transition(self, new_state, detail=""):
        old = self.state.value
        self.state = new_state
        self.state_enter_time = self.get_clock().now()
        self.get_logger().info(f"状态: {old} → {new_state.value} {detail}")
        self._publish_state(detail)

    def _publish_state(self, detail=""):
        m = String()
        m.data = f"{self.state.value}:{detail}" if detail else self.state.value
        self.state_pub.publish(m)

    def _publish_complete(self, result):
        m = String()
        m.data = result
        self.complete_pub.publish(m)
        self.get_logger().info(f"→ 发 {self.complete_topic}: {result}")

    # ================================================================
    # 主循环
    # ================================================================
    def on_timer(self):
        s = self.state
        if s == DockingState.WAIT_LINES:
            self._do_wait_lines()
        elif s == DockingState.ALIGNING:
            self._do_aligning()
        elif s == DockingState.FALLBACK:
            self._do_fallback()

    # -------- WAIT_LINES --------
    def _do_wait_lines(self):
        now = self.get_clock().now()
        elapsed = (now - self.wait_start_time).nanoseconds / 1e9

        target = self._find_target_line()
        if target is not None:
            self.align_start_time = now
            self._transition(DockingState.ALIGNING, "反光条对准中")
            return

        if elapsed >= self.wait_lines_timeout:
            self.get_logger().warn(
                f"等待反光条超时 ({self.wait_lines_timeout}s)，回退直线")
            self._start_fallback()
            return

        self._publish_zero()

    # -------- ALIGNING --------
    def _do_aligning(self):
        now = self.get_clock().now()
        align_elapsed = (now - self.align_start_time).nanoseconds / 1e9

        if align_elapsed > self.align_overall_timeout:
            self.get_logger().warn(
                f"对准超时 ({self.align_overall_timeout}s)，回退直线")
            self._start_fallback()
            return

        target = self._find_target_line()
        if target is None:
            # 反光条丢失
            if (self.lines_received_time is None or
                    (now - self.lines_received_time).nanoseconds / 1e9 >
                    self.line_lost_timeout):
                self.get_logger().warn(
                    f"反光条丢失 {self.line_lost_timeout}s，回退直线")
                self._start_fallback()
                return
            self._publish_zero()
            return

        mid_x = target["mid_x"]
        mid_y = target["mid_y"]

        lateral_error = mid_y
        distance_error = mid_x - self.target_distance

        # 完成判断
        if (abs(lateral_error) < self.lateral_tolerance and
                abs(distance_error) < self.distance_tolerance):
            self._publish_zero()
            self._transition(DockingState.COMPLETE, "对准完成")
            self._publish_complete("aligned")
            return

        # 控制
        wz = -self.kp_yaw * lateral_error
        wz = max(-self.wz_max, min(self.wz_max, wz))

        if abs(lateral_error) > self.lateral_tolerance * 3:
            # 偏差过大，先原地转
            vx = 0.0
        else:
            vx = self.kp_lateral * distance_error
            vx = max(-self.vx_max, min(self.vx_max, vx))
            if 0.0 < abs(vx) < self.vx_min:
                vx = math.copysign(self.vx_min, vx)

        self._publish_velocity(vx, wz)

    # -------- FALLBACK --------
    def _start_fallback(self):
        self._publish_zero()
        self.fallback_origin_xy = self._get_odom_xy()
        self.fallback_start_time = self.get_clock().now()
        self._transition(DockingState.FALLBACK, "直线回退对接")

    def _do_fallback(self):
        now = self.get_clock().now()
        elapsed = (now - self.fallback_start_time).nanoseconds / 1e9

        if elapsed > self.fallback_timeout:
            self._publish_zero()
            self._transition(DockingState.FAILED, "直线对接超时")
            self._publish_complete("failed")
            return

        if self.fallback_origin_xy is None:
            self.fallback_origin_xy = self._get_odom_xy()
            if self.fallback_origin_xy is None:
                return

        cur = self._get_odom_xy()
        if cur is None:
            return

        dx = cur[0] - self.fallback_origin_xy[0]
        dy = cur[1] - self.fallback_origin_xy[1]
        dist = math.hypot(dx, dy)

        if dist >= self.fallback_dock_distance:
            self._publish_zero()
            self._transition(DockingState.COMPLETE, "直线对接完成")
            self._publish_complete("fallback")
            return

        self._publish_velocity(self.fallback_dock_speed, 0.0)

    # ================================================================
    # 反光条筛选
    # ================================================================
    def _find_target_line(self):
        """从最新消息里选最合适的反光条。"""
        if self.latest_lines is None:
            return None

        best = None
        best_score = float("inf")

        for line in self.latest_lines.lines:
            mid_x = float(line.midpoint.x)
            mid_y = float(line.midpoint.y)
            length = float(line.length)

            if length < self.min_line_length or length > self.max_line_length:
                continue
            if mid_x < self.min_target_x or mid_x > self.max_target_x:
                continue
            if abs(mid_y) > self.max_target_y:
                continue

            score = abs(mid_y) + 0.3 * abs(mid_x - self.target_distance)
            if score < best_score:
                best_score = score
                best = {"mid_x": mid_x, "mid_y": mid_y, "length": length}

        return best

    # ================================================================
    # 辅助
    # ================================================================
    def _get_odom_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame,
                Time(), timeout=Duration(seconds=0.1))
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def _publish_velocity(self, vx, wz):
        m = Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        self.cmd_pub.publish(m)

    def _publish_zero(self):
        self._publish_velocity(0.0, 0.0)


def main():
    rclpy.init()
    node = DockingController()
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()