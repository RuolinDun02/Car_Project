#!/usr/bin/env python3

"""
独立多点巡检节点。

流程：
  IDLE
    → NAVIGATING       Nav2 导航到目标点前方
    → SETTLING         持续发布零速度，等待舵轮回正
    → DOCKING          直线低速前进，模拟精对接
    → DOCKED           发布到达消息，等待下一点
    → UNDOCKING        收到下一点后，先直线后退
    → NAVIGATING       前往下一个点

下一点输入：
  /patrol/next         std_msgs/msg/String
  data 为点号，例如 "1"、"2"

对外输出：
  /patrol/arrived      std_msgs/msg/String
  只有“导航 + 对接”全部完成后，才发布对应点号

状态输出：
  /patrol/state        std_msgs/msg/String

常用命令：
  1. 启动巡检，前往点位 1：
     curl -X POST http://127.0.0.1:8189/api/trigger_nav \
       -H 'Content-Type: application/json' \
       -d '{}'

  2. 点位 1 机械臂任务完成，继续前往点位 2：
     curl -X POST http://127.0.0.1:8189/api/arm_done \
       -H 'Content-Type: application/json' \
       -d '{"arm":"BOTH","result":"ok","point_id":"1"}'

  3. 跳过机械臂桥接，直接发布点位 2：
     ros2 topic pub --once /patrol/next std_msgs/msg/String \
       "{data: '2'}"

说明：
  当前对接和退出均使用 odom 位移判断。
  后续接入反光条点云分割时，只需要替换 DOCKING 部分。
"""

import math
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import String

import tf2_ros
from tf2_ros import TransformException


class PatrolState(Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    SETTLING = "SETTLING"
    DOCKING = "DOCKING"
    DOCKED = "DOCKED"
    UNDOCKING = "UNDOCKING"
    FAILED = "FAILED"


class MultiPointPatrol(Node):

    def __init__(self):
        super().__init__("multi_point_patrol")

        # ------------------------------------------------------------
        # 点位配置
        #
        # point_ids 和 point_poses 一一对应。
        # 每个点的 pose 为最终对接完成位姿：[x, y, yaw]。
        # ------------------------------------------------------------
        self.declare_parameter("point_ids", ["1", "2"])
        self.declare_parameter(
            "point_poses",
            [
                1.0, 1.0, 0.0,
                3.0, 1.0, 0.0,
            ],
        )

        # 最终点与预停靠点之间的距离。
        self.declare_parameter("approach_distance", 0.7)

        # 收到新目标时，用地图坐标核验记录的停靠点是否仍然有效。
        self.declare_parameter("docked_position_tolerance", 0.5)

        # 直线对接参数。
        self.declare_parameter("dock_speed", 0.08)
        self.declare_parameter("dock_distance", 0.7)
        self.declare_parameter("steering_settle_time", 1.5)

        # 离开当前停靠点的直线后退参数。
        self.declare_parameter("undock_speed", 0.10)
        self.declare_parameter("undock_distance", 0.7)

        self.declare_parameter("motion_timeout", 20.0)
        self.declare_parameter("control_rate", 20.0)

        # 直线运动发到 velocity_smoother 的输入。
        # velocity_smoother 再输出 /cmd_vel_smoothed 给底盘桥接节点。
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_smoothed")

        self.declare_parameter("next_topic", "/patrol/next")
        self.declare_parameter("arrived_topic", "/patrol/arrived")
        self.declare_parameter("state_topic", "/patrol/state")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        point_ids = list(self.get_parameter("point_ids").value)
        point_poses = list(self.get_parameter("point_poses").value)

        self.approach_distance = float(
            self.get_parameter("approach_distance").value
        )
        self.docked_position_tolerance = max(0.0, float(
            self.get_parameter("docked_position_tolerance").value
        ))
        self.dock_speed = abs(float(
            self.get_parameter("dock_speed").value
        ))
        self.dock_distance = abs(float(
            self.get_parameter("dock_distance").value
        ))
        self.steering_settle_time = max(0.0, float(
            self.get_parameter("steering_settle_time").value
        ))
        self.undock_speed = abs(float(
            self.get_parameter("undock_speed").value
        ))
        self.undock_distance = abs(float(
            self.get_parameter("undock_distance").value
        ))
        self.motion_timeout = float(
            self.get_parameter("motion_timeout").value
        )
        self.control_rate = float(
            self.get_parameter("control_rate").value
        )

        self.cmd_vel_topic = str(
            self.get_parameter("cmd_vel_topic").value
        )
        self.next_topic = str(
            self.get_parameter("next_topic").value
        )
        self.arrived_topic = str(
            self.get_parameter("arrived_topic").value
        )
        self.state_topic = str(
            self.get_parameter("state_topic").value
        )

        self.map_frame = str(
            self.get_parameter("map_frame").value
        )
        self.odom_frame = str(
            self.get_parameter("odom_frame").value
        )
        self.base_frame = str(
            self.get_parameter("base_frame").value
        )

        if len(point_ids) == 0:
            raise RuntimeError("point_ids 不能为空")

        if len(point_poses) != len(point_ids) * 3:
            raise RuntimeError(
                "point_poses 长度必须等于 point_ids 数量乘以 3"
            )

        self.points = {}

        for index, point_id in enumerate(point_ids):
            offset = index * 3

            self.points[str(point_id)] = {
                "x": float(point_poses[offset]),
                "y": float(point_poses[offset + 1]),
                "yaw": float(point_poses[offset + 2]),
            }

        # ------------------------------------------------------------
        # 运行状态
        # ------------------------------------------------------------
        self.state = PatrolState.IDLE

        # 当前准备前往的目标点。
        self.target_point_id = None

        # 当前已经完成对接的点。
        # None 表示当前没有停靠在任何巡检点。
        self.current_point_id = None

        # 直线运动开始位置和时间。
        self.motion_origin = None
        self.motion_started_at = None

        # Nav2 到达后的舵轮回正等待起始时间。
        self.settle_started_at = None

        # 当前 NavigateToPose goal handle。
        self.nav_goal_handle = None

        # ------------------------------------------------------------
        # TF
        # ------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
        )

        # ------------------------------------------------------------
        # Nav2 Action
        # ------------------------------------------------------------
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )

        # ------------------------------------------------------------
        # 发布和订阅
        # ------------------------------------------------------------
        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.arrived_pub = self.create_publisher(
            String,
            self.arrived_topic,
            10,
        )

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.state_pub = self.create_publisher(
            String,
            self.state_topic,
            state_qos,
        )

        self.next_sub = self.create_subscription(
            String,
            self.next_topic,
            self.on_next,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.on_timer,
        )

        self.publish_state()

        self.get_logger().info("独立多点巡检节点已启动")
        self.get_logger().info(
            f"可用巡检点: {list(self.points.keys())}"
        )
        self.get_logger().info(
            f"下一点话题: {self.next_topic}"
        )
        self.get_logger().info(
            f"到达话题: {self.arrived_topic}"
        )

    # ================================================================
    # 下一点接收
    # ================================================================
    def on_next(self, msg: String):
        point_id = msg.data.strip()

        if point_id not in self.points:
            self.get_logger().error(
                f"未知巡检点: {point_id}，"
                f"可用点位: {list(self.points.keys())}"
            )
            return

        if self.state in (
            PatrolState.NAVIGATING,
            PatrolState.SETTLING,
            PatrolState.DOCKING,
            PatrolState.UNDOCKING,
        ):
            self.get_logger().warn(
                f"当前状态为 {self.state.value}，"
                f"忽略新的下一点 {point_id}"
            )
            return

        # current_point_id 只是上一次完成对接时记录的逻辑状态。
        # 如果底盘被人工移动，需要用 map TF 核验这个状态是否仍然有效。
        if self.current_point_id is not None:
            current_xy = self.get_map_xy()

            if current_xy is None:
                self.get_logger().warn(
                    "无法获取 map 到底盘的 TF，不能确认当前停靠位置；"
                    f"暂不执行前往 {point_id} 号点，请稍后重试"
                )
                return

            recorded_point = self.points[self.current_point_id]
            distance_to_recorded_point = math.hypot(
                current_xy[0] - recorded_point["x"],
                current_xy[1] - recorded_point["y"],
            )

            if distance_to_recorded_point > self.docked_position_tolerance:
                old_point_id = self.current_point_id
                self.current_point_id = None
                self.get_logger().warn(
                    f"底盘距记录的 {old_point_id} 号停靠点 "
                    f"{distance_to_recorded_point:.3f} m，超过 "
                    f"{self.docked_position_tolerance:.3f} m；"
                    "清除旧停靠状态，直接从当前位置导航"
                )
            else:
                self.get_logger().info(
                    f"底盘距记录的 {self.current_point_id} 号停靠点 "
                    f"{distance_to_recorded_point:.3f} m，停靠状态有效"
                )

        # 确实停靠在同一个点，直接报告到达。
        if self.current_point_id == point_id:
            self.get_logger().info(
                f"当前确实停靠在 {point_id} 号点"
            )
            self.publish_arrived(point_id)
            return

        self.target_point_id = point_id

        # 经 TF 确认仍停靠在其他点：必须先退出对接。
        if self.current_point_id is not None:
            self.start_relative_motion(PatrolState.UNDOCKING)
            return

        # 当前没有停靠：直接开始导航。
        self.start_navigation(point_id)

    # ================================================================
    # Nav2 导航
    # ================================================================
    def start_navigation(self, point_id: str):
        point = self.points[point_id]

        # point 表示最终对接位姿。
        # Nav2 目标设置在该点后方 approach_distance 米处。
        staging_x = (
            point["x"]
            - self.approach_distance * math.cos(point["yaw"])
        )
        staging_y = (
            point["y"]
            - self.approach_distance * math.sin(point["yaw"])
        )

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.fail("NavigateToPose action server 不可用")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.map_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = staging_x
        goal_pose.pose.position.y = staging_y
        goal_pose.pose.position.z = 0.0

        goal_pose.pose.orientation.z = math.sin(
            point["yaw"] / 2.0
        )
        goal_pose.pose.orientation.w = math.cos(
            point["yaw"] / 2.0
        )

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        self.set_state(
            PatrolState.NAVIGATING,
            f"前往 {point_id} 号点预停靠位置",
        )

        self.get_logger().info(
            f"开始前往 {point_id} 号点"
        )
        self.get_logger().info(
            f"预停靠点 A（最终点沿对接方向反推 "
            f"{self.approach_distance:.3f} m）: "
            f"x={staging_x:.3f}, y={staging_y:.3f}, "
            f"yaw={point['yaw']:.4f} rad "
            f"({math.degrees(point['yaw']):.1f} deg)"
        )

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.on_nav_goal_response)

    def on_nav_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exception:
            self.fail(f"发送导航目标异常: {exception}")
            return

        if not goal_handle.accepted:
            self.fail("Nav2 拒绝导航目标")
            return

        self.nav_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        try:
            wrapped_result = future.result()
        except Exception as exception:
            self.fail(f"读取导航结果异常: {exception}")
            return

        self.nav_goal_handle = None

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.fail(
                f"导航失败，status={wrapped_result.status}"
            )
            return

        self.get_logger().info(
            f"已到达 {self.target_point_id} 号点附近，"
            "等待舵轮回正"
        )

        self.start_steering_settle()

    # ================================================================
    # 对接和退出对接
    # ================================================================
    def start_steering_settle(self):
        self.publish_zero()
        self.settle_started_at = self.get_clock().now()
        self.set_state(
            PatrolState.SETTLING,
            f"持续零速度 {self.steering_settle_time:.1f} 秒，"
            "等待舵轮回正",
        )

    def start_relative_motion(self, new_state: PatrolState):
        self.publish_zero()

        self.motion_origin = self.get_odom_xy()
        self.motion_started_at = self.get_clock().now()

        if new_state == PatrolState.DOCKING:
            detail = (
                f"向前对接 {self.target_point_id} 号点"
            )
        else:
            detail = (
                f"退出 {self.current_point_id} 号点"
            )

        self.set_state(new_state, detail)

    def on_timer(self):
        if self.state == PatrolState.SETTLING:
            # 每个控制周期都发零速度，而不是只发一次，
            # 给底盘足够时间将舵轮回正。
            self.publish_zero()

            elapsed = (
                self.get_clock().now() - self.settle_started_at
            ).nanoseconds / 1e9

            if elapsed >= self.steering_settle_time:
                self.settle_started_at = None
                self.get_logger().info(
                    "舵轮回正等待完成，开始直线对接"
                )
                self.start_relative_motion(PatrolState.DOCKING)
            return

        if self.state not in (
            PatrolState.DOCKING,
            PatrolState.UNDOCKING,
        ):
            return

        elapsed = (
            self.get_clock().now() - self.motion_started_at
        ).nanoseconds / 1e9

        if elapsed > self.motion_timeout:
            self.publish_zero()
            self.fail(
                f"{self.state.value} 超时，"
                f"超过 {self.motion_timeout:.1f} 秒"
            )
            return

        current_xy = self.get_odom_xy()

        if current_xy is None:
            return

        if self.motion_origin is None:
            self.motion_origin = current_xy
            return

        dx = current_xy[0] - self.motion_origin[0]
        dy = current_xy[1] - self.motion_origin[1]
        distance = math.hypot(dx, dy)

        if self.state == PatrolState.DOCKING:
            if distance >= self.dock_distance:
                self.finish_docking()
                return

            self.publish_velocity(self.dock_speed)
            return

        if self.state == PatrolState.UNDOCKING:
            if distance >= self.undock_distance:
                self.finish_undocking()
                return

            self.publish_velocity(-self.undock_speed)

    def finish_docking(self):
        self.publish_zero()

        point_id = self.target_point_id
        self.current_point_id = point_id

        self.motion_origin = None
        self.motion_started_at = None

        self.set_state(
            PatrolState.DOCKED,
            f"已完成 {point_id} 号点对接",
        )

        # 只有对接完成后才发布到达消息。
        self.publish_arrived(point_id)

    def finish_undocking(self):
        self.publish_zero()

        old_point_id = self.current_point_id
        self.current_point_id = None

        self.motion_origin = None
        self.motion_started_at = None

        self.get_logger().info(
            f"已退出 {old_point_id} 号点，"
            f"准备前往 {self.target_point_id} 号点"
        )

        next_point_id = self.target_point_id
        self.start_navigation(next_point_id)

    # ================================================================
    # 发布
    # ================================================================
    def publish_velocity(self, linear_x: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def publish_zero(self):
        self.publish_velocity(0.0)

    def publish_arrived(self, point_id: str):
        msg = String()
        msg.data = point_id
        self.arrived_pub.publish(msg)

        self.get_logger().info(
            f"发布到达消息: {point_id}"
        )

    def set_state(
        self,
        state: PatrolState,
        detail: str = "",
    ):
        self.state = state
        self.publish_state(detail)

        if detail:
            self.get_logger().info(
                f"状态: {state.value}，{detail}"
            )
        else:
            self.get_logger().info(
                f"状态: {state.value}"
            )

    def publish_state(self, detail: str = ""):
        msg = String()

        if detail:
            msg.data = f"{self.state.value}:{detail}"
        else:
            msg.data = self.state.value

        self.state_pub.publish(msg)

    def fail(self, reason: str):
        self.publish_zero()
        self.set_state(PatrolState.FAILED, reason)
        self.get_logger().error(reason)

    # ================================================================
    # TF
    # ================================================================
    def get_map_xy(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None

        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def get_odom_xy(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None

        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def stop(self):
        self.publish_zero()


def main():
    rclpy.init()
    node = MultiPointPatrol()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
