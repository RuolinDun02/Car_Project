#!/usr/bin/env python3
"""由 Nav2 BT 调用的脱困 Action Server。

published costmap 是只读输出，向它发布 OccupancyGrid 不能修改 planner_server
内部地图。因此“用局部实时图覆盖全局局部区域”采用 Nav2 支持的等价实现：
按 local costmap 窗口大小清除 global costmap 的可重置层，再等待订阅同一
/scan 的 global obstacle_layer 重新填充真实障碍。static/keepout 会保留或重建。

本节点不主动抢占 Nav2，也不发送 FollowPath。BT 完成一次常规恢复后调用本
action；action 成功后，BT 使用同一 goal 和同一条原始 path 继续 FollowPath。
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import AssistedTeleop
from nav2_msgs.srv import ClearCostmapAroundRobot
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class StuckEscapeNode(Node):

    def __init__(self):
        super().__init__("stuck_escape_node")

        self.declare_parameter("enable", True)
        self.declare_parameter("action_name", "stuck_escape")
        self.declare_parameter("costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("plan_topic", "/plan")
        # 发给 velocity_smoother 输入，不绕过平滑器。
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter(
            "global_clear_service",
            "/global_costmap/clear_around_global_costmap",
        )
        self.declare_parameter("escape_cost_threshold", 80)
        self.declare_parameter("safe_cost_threshold", 30)
        self.declare_parameter("scan_radius", 0.8)
        self.declare_parameter("scan_angles", 24)
        self.declare_parameter("robot_radius", 0.35)
        self.declare_parameter("escape_speed", 0.08)
        self.declare_parameter("rotate_speed", 0.18)
        self.declare_parameter("heading_tolerance", 0.25)
        self.declare_parameter("min_escape_distance", 0.15)
        self.declare_parameter("escape_duration_max", 5.0)
        self.declare_parameter("retry_wait_time", 1.0)
        # global costmap 当前为 1 Hz，至少等待一次新 scan 被融合。
        self.declare_parameter("global_refresh_wait", 1.2)
        self.declare_parameter("path_bias_weight", 20.0)
        self.declare_parameter("path_lookahead", 0.6)

        p = self.get_parameter
        self.enable = bool(p("enable").value)
        self.action_name = str(p("action_name").value)
        self.costmap_topic = str(p("costmap_topic").value)
        self.plan_topic = str(p("plan_topic").value)
        self.cmd_vel_topic = str(p("cmd_vel_topic").value)
        self.robot_frame = str(p("robot_frame").value)
        self.global_clear_service = str(p("global_clear_service").value)
        self.escape_cost_thr = int(p("escape_cost_threshold").value)
        self.safe_cost_thr = int(p("safe_cost_threshold").value)
        self.scan_radius = float(p("scan_radius").value)
        self.scan_angles = int(p("scan_angles").value)
        self.robot_radius = float(p("robot_radius").value)
        self.escape_speed = float(p("escape_speed").value)
        self.rotate_speed = float(p("rotate_speed").value)
        self.heading_tolerance = float(p("heading_tolerance").value)
        self.min_escape_distance = float(p("min_escape_distance").value)
        self.escape_dur_max = float(p("escape_duration_max").value)
        self.retry_wait_time = float(p("retry_wait_time").value)
        self.global_refresh_wait = float(p("global_refresh_wait").value)
        self.path_bias_weight = float(p("path_bias_weight").value)
        self.path_lookahead = float(p("path_lookahead").value)

        self.costmap = None
        self.reference_plan = None
        self._goal_active = False
        self._goal_lock = threading.Lock()
        self.callback_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self.costmap_topic, self._on_costmap, latched_qos,
            callback_group=self.callback_group)
        self.create_subscription(
            Path, self.plan_topic, self._on_plan, 10,
            callback_group=self.callback_group)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(
            String, "/stuck_escape/state", latched_qos)
        self.clear_global_around = self.create_client(
            ClearCostmapAroundRobot, self.global_clear_service,
            callback_group=self.callback_group)
        self.action_server = ActionServer(
            self, AssistedTeleop, self.action_name,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self.callback_group)

        self._publish_state("IDLE")
        self.get_logger().info(
            f"stuck_escape action 已启动: /{self.action_name}；"
            f"速度输出={self.cmd_vel_topic}")

    def _on_costmap(self, msg: OccupancyGrid):
        self.costmap = msg

    def _on_plan(self, msg: Path):
        if msg.poses:
            # 修改后的 BT 每个目标只规划一次，这是本任务的原始参考路径。
            self.reference_plan = msg

    def _on_goal(self, _goal_request):
        if not self.enable:
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_active:
                self.get_logger().warn("已有脱困任务运行，拒绝重复 action")
                return GoalResponse.REJECT
            self._goal_active = True
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        started_at = time.monotonic()
        result = AssistedTeleop.Result()
        try:
            if not self._wait_for_inputs(goal_handle, 3.0):
                if goal_handle.is_cancel_requested:
                    return self._finish_cancel(goal_handle, result, started_at)
                return self._finish_error(
                    goal_handle, result, AssistedTeleop.Result.TF_ERROR,
                    "等待 local costmap/TF 超时", started_at)

            self._publish_state("REFRESHING_GLOBAL_WINDOW")
            self._refresh_global_window(goal_handle)
            if goal_handle.is_cancel_requested:
                return self._finish_cancel(goal_handle, result, started_at)

            pose = self._get_robot_pose()
            if pose is None:
                return self._finish_error(
                    goal_handle, result, AssistedTeleop.Result.TF_ERROR,
                    "无法取得机器人在 local costmap 中的位姿", started_at)

            if self._robot_cost(pose) < self.escape_cost_thr:
                self.get_logger().info("实时局部图显示起点已可通行，交回原路径")
                return self._finish_success(goal_handle, result, started_at)

            self._publish_state("ESCAPING")
            return self._escape_until_safe(goal_handle, result, started_at)
        finally:
            self._stop()
            with self._goal_lock:
                self._goal_active = False

    def _escape_until_safe(self, goal_handle, result, action_started_at):
        cycle_started_at = time.monotonic()
        start_pose = self._get_robot_pose()
        last_direction_update = 0.0
        direction_world = None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._finish_cancel(
                    goal_handle, result, action_started_at)

            pose = self._get_robot_pose()
            if pose is None:
                self._stop()
                time.sleep(0.1)
                continue

            moved = 0.0
            if start_pose is not None:
                moved = math.hypot(
                    pose[0] - start_pose[0], pose[1] - start_pose[1])
            cost = self._robot_cost(pose)
            if 0 <= cost < self.safe_cost_thr and (
                    moved >= self.min_escape_distance or cost == 0):
                self.get_logger().info(
                    f"脱困完成: cost={cost}, moved={moved:.3f} m；恢复原路径")
                return self._finish_success(
                    goal_handle, result, action_started_at)

            now = time.monotonic()
            if now - last_direction_update >= 0.5 or direction_world is None:
                direction_world = self._find_escape_direction(pose)
                last_direction_update = now

            if direction_world is None:
                # 无安全方向时保持 action RUNNING，不把可恢复问题变成 status=6。
                self._publish_state("WAITING_FOR_SAFE_DIRECTION")
                self._stop()
                time.sleep(self.retry_wait_time)
                self._refresh_global_window(goal_handle)
                self._publish_state("ESCAPING")
                cycle_started_at = time.monotonic()
                start_pose = self._get_robot_pose()
                continue

            if now - cycle_started_at > self.escape_dur_max:
                # 单轮运动到上限后停住、刷新感知，再安全地开始下一轮。
                self._publish_state("ESCAPE_REASSESS")
                self._stop()
                time.sleep(self.retry_wait_time)
                self._refresh_global_window(goal_handle)
                cycle_started_at = time.monotonic()
                start_pose = self._get_robot_pose()
                direction_world = None
                self._publish_state("ESCAPING")
                continue

            self._publish_escape_command(pose[2], direction_world)
            feedback = AssistedTeleop.Feedback()
            feedback.current_teleop_duration = Duration(
                seconds=now - action_started_at).to_msg()
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)

        return self._finish_error(
            goal_handle, result, AssistedTeleop.Result.UNKNOWN,
            "rclpy 已停止", action_started_at)

    def _wait_for_inputs(self, goal_handle, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            if self.costmap is not None and self._get_robot_pose() is not None:
                return True
            time.sleep(0.05)
        return False

    def _refresh_global_window(self, goal_handle):
        if self.costmap is None:
            return
        if not self.clear_global_around.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"服务不可用: {self.global_clear_service}；仅等待雷达刷新")
        else:
            request = ClearCostmapAroundRobot.Request()
            # reset_distance 是清理正方形的边长；取 local 窗口短边。
            request.reset_distance = float(min(
                self.costmap.info.width * self.costmap.info.resolution,
                self.costmap.info.height * self.costmap.info.resolution))
            future = self.clear_global_around.call_async(request)
            deadline = time.monotonic() + 2.0
            while (rclpy.ok() and not future.done()
                   and time.monotonic() < deadline
                   and not goal_handle.is_cancel_requested):
                time.sleep(0.02)
            if future.done() and future.exception() is None:
                self.get_logger().info(
                    f"已清理全局动态窗口 {request.reset_distance:.2f} m，"
                    "等待实时雷达重填")
            else:
                self.get_logger().warn("清理全局动态窗口未完成")

        deadline = time.monotonic() + self.global_refresh_wait
        while (rclpy.ok() and time.monotonic() < deadline
               and not goal_handle.is_cancel_requested):
            self._stop()
            time.sleep(0.05)

    def _get_robot_pose(self):
        if self.costmap is None:
            return None
        frame = self.costmap.header.frame_id or "odom"
        try:
            tf = self.tf_buffer.lookup_transform(
                frame, self.robot_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (tf.transform.translation.x, tf.transform.translation.y, yaw)

    def _world_to_grid(self, wx, wy):
        if self.costmap is None:
            return None
        origin = self.costmap.info.origin
        resolution = self.costmap.info.resolution
        gx = int(math.floor((wx - origin.position.x) / resolution))
        gy = int(math.floor((wy - origin.position.y) / resolution))
        if (gx < 0 or gy < 0 or gx >= self.costmap.info.width
                or gy >= self.costmap.info.height):
            return None
        return gx, gy

    def _cost_at(self, gx, gy):
        if self.costmap is None:
            return -1
        if (gx < 0 or gy < 0 or gx >= self.costmap.info.width
                or gy >= self.costmap.info.height):
            return -1
        return self.costmap.data[gy * self.costmap.info.width + gx]

    def _robot_cost(self, pose):
        grid = self._world_to_grid(pose[0], pose[1])
        return -1 if grid is None else self._cost_at(*grid)

    def _find_escape_direction(self, pose):
        """在 odom costmap 中选安全低代价方向，并偏向原路径前方。"""
        grid = self._world_to_grid(pose[0], pose[1])
        if grid is None or self.costmap is None:
            return None

        resolution = self.costmap.info.resolution
        radius_cells = max(1, int(self.scan_radius / resolution))
        footprint_cells = max(1, int(self.robot_radius / resolution))
        preferred = self._preferred_path_direction(pose)
        best = None

        for i in range(self.scan_angles):
            angle = 2.0 * math.pi * i / self.scan_angles
            total_cost = 0.0
            samples = 0
            blocked = False
            for radius in range(1, radius_cells + 1):
                cx = grid[0] + int(round(radius * math.cos(angle)))
                cy = grid[1] + int(round(radius * math.sin(angle)))
                lateral = min(footprint_cells, radius)
                for offset in range(-lateral, lateral + 1):
                    sx = cx + int(round(offset * -math.sin(angle)))
                    sy = cy + int(round(offset * math.cos(angle)))
                    cost = self._cost_at(sx, sy)
                    if cost < 0 or cost >= self.escape_cost_thr:
                        blocked = True
                        break
                    total_cost += cost
                    samples += 1
                if blocked:
                    break
            if blocked or samples == 0:
                continue

            score = total_cost / samples
            if preferred is not None:
                score += self.path_bias_weight * abs(
                    normalize_angle(angle - preferred)) / math.pi
            if best is None or score < best[0]:
                best = (score, angle)

        return None if best is None else best[1]

    def _preferred_path_direction(self, robot_pose):
        plan = self.reference_plan
        if plan is None or not plan.poses:
            return None
        costmap_frame = self.costmap.header.frame_id or "odom"
        plan_frame = plan.header.frame_id or "map"
        try:
            tf = self.tf_buffer.lookup_transform(
                costmap_frame, plan_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception:
            return None

        q = tf.transform.rotation
        tf_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_yaw = math.cos(tf_yaw)
        sin_yaw = math.sin(tf_yaw)
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        points = []
        nearest_idx = 0
        nearest_dist = float("inf")
        for index, stamped in enumerate(plan.poses):
            px = stamped.pose.position.x
            py = stamped.pose.position.y
            wx = tx + cos_yaw * px - sin_yaw * py
            wy = ty + sin_yaw * px + cos_yaw * py
            points.append((wx, wy))
            distance = math.hypot(wx - robot_pose[0], wy - robot_pose[1])
            if distance < nearest_dist:
                nearest_dist = distance
                nearest_idx = index

        target_idx = nearest_idx
        accumulated = 0.0
        while (target_idx + 1 < len(points)
               and accumulated < self.path_lookahead):
            accumulated += math.hypot(
                points[target_idx + 1][0] - points[target_idx][0],
                points[target_idx + 1][1] - points[target_idx][1])
            target_idx += 1
        target = points[target_idx]
        if math.hypot(target[0] - robot_pose[0],
                      target[1] - robot_pose[1]) < 0.05:
            return None
        return math.atan2(
            target[1] - robot_pose[1], target[0] - robot_pose[0])

    def _publish_escape_command(self, robot_yaw, direction_world):
        heading_error = normalize_angle(direction_world - robot_yaw)
        command = Twist()
        if abs(heading_error) > self.heading_tolerance:
            command.angular.z = math.copysign(self.rotate_speed, heading_error)
        else:
            command.linear.x = self.escape_speed
        self.cmd_pub.publish(command)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_state(self, state):
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    @staticmethod
    def _set_elapsed(result, started_at):
        result.total_elapsed_time = Duration(
            seconds=max(0.0, time.monotonic() - started_at)).to_msg()

    def _finish_success(self, goal_handle, result, started_at):
        self._stop()
        result.error_code = AssistedTeleop.Result.NONE
        result.error_msg = ""
        self._set_elapsed(result, started_at)
        goal_handle.succeed()
        self._publish_state("SUCCEEDED")
        return result

    def _finish_cancel(self, goal_handle, result, started_at):
        self._stop()
        result.error_code = AssistedTeleop.Result.NONE
        result.error_msg = "脱困任务被取消"
        self._set_elapsed(result, started_at)
        goal_handle.canceled()
        self._publish_state("CANCELED")
        return result

    def _finish_error(self, goal_handle, result, code, message, started_at):
        self._stop()
        result.error_code = code
        result.error_msg = message
        self._set_elapsed(result, started_at)
        goal_handle.abort()
        self._publish_state(f"FAILED:{message}")
        self.get_logger().error(message)
        return result

    def destroy_node(self):
        self._stop()
        self.action_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StuckEscapeNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
