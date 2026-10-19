#!/usr/bin/env python3

"""
独立多点巡检节点（对接交给 docking_controller）。

流程：
  IDLE
    → INITIAL_BACKUP   第一次导航前直线后退
    → INITIAL_SPIN_360 第一次导航前原地旋转 360 度
    → INITIAL_WAIT     旋转完成后短暂停稳
    → NAVIGATING       Nav2 导航到目标点前方
    → SETTLING         持续发布零速度，等待舵轮回正
    → DOCKING          请求 docking_controller 对接，等 /docking/complete
    → DOCKED           发布到达消息，等待下一点
    → UNDOCKING        收到下一点后，先直线后退
    → NAVIGATING       前往下一个点

点位同步：
  /patrol/current_point 发布 base_footprint 当前位于哪个配置点附近。
  只比较 map 中的二维位置，不检查航向角。空字符串表示不在任何点位附近。

下一点输入：
  /patrol/next         std_msgs/msg/String
  数据为点号，如 "1"、"2"

对接接口：
  /docking/request     std_msgs/msg/String   发 "start"（或点号）
  /docking/complete    std_msgs/msg/String   收 "aligned"/"fallback"/"failed"

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

  3. 跳过机械臂动作，通过 HTTP 直接发布点位 2：
     curl -X POST http://127.0.0.1:8189/api/trigger_nav \
       -H 'Content-Type: application/json' \
       -d '{"point_id":"2"}'

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
    INITIAL_BACKUP = "INITIAL_BACKUP"
    INITIAL_SPIN_360 = "INITIAL_SPIN_360"
    INITIAL_WAIT = "INITIAL_WAIT"
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

        # 定期根据 map -> base_footprint 同步所在点位的间隔。
        self.declare_parameter("docked_verify_interval", 1.0)

        # 节点启动后的第一次导航前，先后退、旋转一周并短暂停稳。
        self.declare_parameter("initial_backup_distance", 0.3)
        self.declare_parameter("initial_backup_speed", 0.15)
        self.declare_parameter("initial_backup_timeout", 10.0)
        self.declare_parameter("initial_spin_speed_deg", 60.0)
        self.declare_parameter("initial_spin_timeout", 60.0)
        self.declare_parameter("initial_wait_after_spin", 2.0)

        # 舵轮回正等待
        self.declare_parameter("steering_settle_time", 1.5)

        # 离开当前停靠点的直线后退参数。
        self.declare_parameter("undock_speed", 0.10)
        self.declare_parameter("undock_distance", 0.3)

        self.declare_parameter("motion_timeout", 20.0)
        # ★ 对接总超时（docking_controller 内部有自己的超时）
        self.declare_parameter("docking_timeout", 60.0)
        self.declare_parameter("control_rate", 20.0)

        # 直线运动发到 velocity_smoother 的输入。
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_smoothed")

        self.declare_parameter("next_topic", "/patrol/next")
        self.declare_parameter("arrived_topic", "/patrol/arrived")
        self.declare_parameter("state_topic", "/patrol/state")
        self.declare_parameter(
            "current_point_topic",
            "/patrol/current_point",
        )

        # ★ 对接控制器接口
        self.declare_parameter("docking_request_topic", "/docking/request")
        self.declare_parameter("docking_complete_topic", "/docking/complete")

        # ★ 外部干预检测
        self.declare_parameter("external_goal_topic", "/goal_pose")

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
        self.docked_verify_interval = max(0.1, float(
            self.get_parameter("docked_verify_interval").value
        ))
        self.initial_backup_distance = abs(float(
            self.get_parameter("initial_backup_distance").value
        ))
        self.initial_backup_speed = abs(float(
            self.get_parameter("initial_backup_speed").value
        ))
        self.initial_backup_timeout = max(0.0, float(
            self.get_parameter("initial_backup_timeout").value
        ))
        self.initial_spin_speed_deg = abs(float(
            self.get_parameter("initial_spin_speed_deg").value
        ))
        self.initial_spin_timeout = max(0.0, float(
            self.get_parameter("initial_spin_timeout").value
        ))
        self.initial_wait_after_spin = max(0.0, float(
            self.get_parameter("initial_wait_after_spin").value
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
        self.docking_timeout = float(
            self.get_parameter("docking_timeout").value
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
        self.current_point_topic = str(
            self.get_parameter("current_point_topic").value
        )
        self.docking_request_topic = str(
            self.get_parameter("docking_request_topic").value
        )
        self.docking_complete_topic = str(
            self.get_parameter("docking_complete_topic").value
        )
        self.external_goal_topic = str(
            self.get_parameter("external_goal_topic").value
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

        self.target_point_id = None
        # 已经完成对接的点位，供巡检流程判断是否需要退出。
        self.current_point_id = None
        # 仅表示二维位置位于哪个配置点附近，不代表已完成对接。
        self.located_point_id = None

        self.initial_prepare_completed = False
        self.initial_stage_started_at = None
        self.initial_backup_origin = None
        self.initial_last_yaw = None
        self.initial_accumulated_yaw = 0.0

        self.motion_origin = None
        self.motion_started_at = None

        self.settle_started_at = None

        # ★ 对接阶段起始时间
        self.docking_started_at = None

        # 根据 map -> base_footprint 定期同步所在点位。
        self.last_docked_verify_at = None

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

        # ★ 对接请求发布者
        self.docking_request_pub = self.create_publisher(
            String,
            self.docking_request_topic,
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

        self.current_point_pub = self.create_publisher(
            String,
            self.current_point_topic,
            state_qos,
        )

        self.next_sub = self.create_subscription(
            String,
            self.next_topic,
            self.on_next,
            10,
        )

        # ★ 对接完成订阅
        self.docking_complete_sub = self.create_subscription(
            String,
            self.docking_complete_topic,
            self.on_docking_complete,
            10,
        )

        # ★ 订阅 /goal_pose 检测外部干预（Foxglove/RViz）
        #   注意：multi_point_patrol 自己用 NavigateToPose action
        #        发目标，不走 /goal_pose 话题，不会误触发
        self.external_goal_sub = self.create_subscription(
            PoseStamped,
            self.external_goal_topic,
            self.on_external_goal,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.on_timer,
        )

        self.publish_state()
        self.publish_located_point(None, "节点启动", force=True)

        self.get_logger().info("=" * 60)
        self.get_logger().info("独立多点巡检节点已启动（对接由 docking_controller 执行）")
        self.get_logger().info(
            f"可用巡检点: {list(self.points.keys())}"
        )
        self.get_logger().info(
            f"下一点话题: {self.next_topic}"
        )
        self.get_logger().info(
            f"到达话题: {self.arrived_topic}"
        )
        self.get_logger().info(
            f"对接请求: {self.docking_request_topic}"
        )
        self.get_logger().info(
            f"对接完成: {self.docking_complete_topic}"
        )
        self.get_logger().info(
            f"外部干预检测: {self.external_goal_topic}"
        )
        self.get_logger().info(
            f"当前点位同步: {self.current_point_topic}"
        )
        self.get_logger().info("=" * 60)

    # ================================================================
    # ★ 外部 goal_pose 检测
    # ================================================================
    def on_external_goal(self, msg: PoseStamped):
        """检测到外部发 /goal_pose。

        场景：
          - Foxglove / RViz 手动发目标点
          - 其它节点（非本节点）发目标点

        注意：
          - multi_point_patrol 自己用 NavigateToPose action 发目标，
            不走 /goal_pose 话题 → 不会误触发
        """
        # 正在执行巡检任务时，外部干预要忽略（避免打断）
        if self.state in (
            PatrolState.NAVIGATING,
            PatrolState.DOCKING,
            PatrolState.UNDOCKING,
            PatrolState.INITIAL_BACKUP,
            PatrolState.INITIAL_SPIN_360,
            PatrolState.INITIAL_WAIT,
        ):
            self.get_logger().warn(
                f"检测到外部 /goal_pose，但当前状态 "
                f"{self.state.value} 正在执行，忽略"
            )
            return

        # 发布目标不等于机器人已离开；后续由实际坐标更新点位。
        self.get_logger().info(
            "检测到外部 /goal_pose，"
            "后续根据 map 中的实际位置更新当前点位"
        )

        # 状态切回 IDLE，避免继续执行对接流程。
        if self.state in (
            PatrolState.DOCKED,
            PatrolState.SETTLING,
            PatrolState.FAILED,
        ):
            self.set_state(PatrolState.IDLE, "外部 /goal_pose 干预")

    # ================================================================
    # 下一点接收
    # ================================================================
    def on_next(self, msg: String):
        point_id = msg.data.strip()

        if point_id not in self.points:
            self.get_logger().error(
                f"下一点处理失败；下一点位={point_id}；"
                f"未知巡检点，"
                f"可用点位: {list(self.points.keys())}"
            )
            return

        if self.state in (
            PatrolState.INITIAL_BACKUP,
            PatrolState.INITIAL_SPIN_360,
            PatrolState.INITIAL_WAIT,
            PatrolState.NAVIGATING,
            PatrolState.SETTLING,
            PatrolState.DOCKING,
            PatrolState.UNDOCKING,
        ):
            self.get_logger().warn(
                f"下一点处理失败；下一点位={point_id}；"
                f"当前状态为 {self.state.value}，忽略请求"
            )
            return

        if self.current_point_id is not None:
            current_xy = self.get_map_xy()

            if current_xy is None:
                # ★ 修改：TF 不可用 → 保守清除停靠状态，继续导航
                #   （原逻辑是直接 return，导致用户请求被"吃掉"）
                self.get_logger().warn(
                    f"下一点处理；下一点位={point_id}；"
                    "无法获取 map 到底盘的 TF，"
                    "保守清除停靠状态，直接从当前位置导航"
                )
                self.current_point_id = None
            else:
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

        if self.current_point_id == point_id:
            self.get_logger().info(
                f"下一点处理成功；下一点位={point_id}；"
                "当前已经停靠在该点"
            )
            self.publish_arrived(point_id)
            return

        self.target_point_id = point_id

        if self.current_point_id is not None:
            self.get_logger().info(
                f"下一点处理成功；下一点位={point_id}；"
                f"先退出 {self.current_point_id} 号点"
            )
            self.start_relative_motion(PatrolState.UNDOCKING)
            return

        if not self.initial_prepare_completed:
            self.get_logger().info(
                f"下一点处理成功；下一点位={point_id}；"
                "开始首次导航前准备"
            )
            self.start_initial_backup()
            return

        self.start_navigation(point_id)

    # ================================================================
    # ★ 对接完成回调
    # ================================================================
    def on_docking_complete(self, msg: String):
        """收到 docking_controller 的完成信号。"""
        if self.state != PatrolState.DOCKING:
            self.get_logger().warn(
                f"当前状态 {self.state.value}，忽略对接完成信号"
            )
            return

        result = msg.data.strip()
        self.get_logger().info(f"收到对接完成信号: {result}")

        if result in ("aligned", "fallback"):
            self.finish_docking()
        else:
            self.fail(f"对接失败: {result}")

    # ================================================================
    # 第一次导航前准备
    # ================================================================
    def start_initial_backup(self):
        self.publish_zero()
        self.initial_stage_started_at = self.get_clock().now()

        pose = self.get_map_pose()
        if pose is None:
            self.initial_backup_origin = None
        else:
            self.initial_backup_origin = (pose[0], pose[1])

        self.set_state(
            PatrolState.INITIAL_BACKUP,
            f"第一次导航前后退 {self.initial_backup_distance:.3f} m；"
            f"下一点位={self.target_point_id}",
        )

    def start_initial_spin(self):
        self.publish_zero()
        self.initial_stage_started_at = self.get_clock().now()

        pose = self.get_map_pose()
        self.initial_last_yaw = None if pose is None else pose[2]
        self.initial_accumulated_yaw = 0.0

        self.set_state(
            PatrolState.INITIAL_SPIN_360,
            f"第一次导航前旋转 360 度；"
            f"下一点位={self.target_point_id}",
        )

    def start_initial_wait(self):
        self.publish_zero()
        self.initial_stage_started_at = self.get_clock().now()
        self.set_state(
            PatrolState.INITIAL_WAIT,
            f"旋转完成后等待 {self.initial_wait_after_spin:.1f} 秒；"
            f"下一点位={self.target_point_id}",
        )

    def finish_initial_prepare(self):
        self.publish_zero()
        self.initial_prepare_completed = True
        self.initial_stage_started_at = None
        self.initial_backup_origin = None
        self.initial_last_yaw = None

        point_id = self.target_point_id
        if point_id is None:
            self.fail("首次导航准备完成，但下一点位为空")
            return

        self.get_logger().info(
            f"首次导航前准备完成；下一点位={point_id}"
        )
        self.start_navigation(point_id)

    def do_initial_backup(self):
        if self.initial_stage_elapsed() > self.initial_backup_timeout:
            self.get_logger().warn(
                f"首次后退超时；下一点位={self.target_point_id}；"
                "继续执行旋转"
            )
            self.start_initial_spin()
            return

        pose = self.get_map_pose()
        if pose is None:
            self.get_logger().warn(
                f"首次后退无法获取位姿；下一点位={self.target_point_id}；"
                "跳过后退，继续执行旋转"
            )
            self.start_initial_spin()
            return

        if self.initial_backup_origin is None:
            self.initial_backup_origin = (pose[0], pose[1])
            return

        dx = pose[0] - self.initial_backup_origin[0]
        dy = pose[1] - self.initial_backup_origin[1]
        distance = math.hypot(dx, dy)

        if distance >= self.initial_backup_distance:
            self.get_logger().info(
                f"首次后退完成 ({distance:.3f} m)；"
                f"下一点位={self.target_point_id}"
            )
            self.start_initial_spin()
            return

        self.publish_velocity(-self.initial_backup_speed)

    def do_initial_spin(self):
        if self.initial_stage_elapsed() > self.initial_spin_timeout:
            self.get_logger().warn(
                "首次旋转 360 度超时"
                f"（累计 {math.degrees(self.initial_accumulated_yaw):.1f} 度）；"
                f"下一点位={self.target_point_id}；继续导航"
            )
            self.start_initial_wait()
            return

        pose = self.get_map_pose()
        if pose is None:
            self.get_logger().warn(
                f"首次旋转无法获取位姿；下一点位={self.target_point_id}；"
                "跳过旋转，继续导航"
            )
            self.start_initial_wait()
            return

        current_yaw = pose[2]
        if self.initial_last_yaw is None:
            self.initial_last_yaw = current_yaw
            return

        delta = self.normalize_angle(current_yaw - self.initial_last_yaw)
        self.initial_accumulated_yaw += delta
        self.initial_last_yaw = current_yaw

        if abs(self.initial_accumulated_yaw) >= 2.0 * math.pi:
            self.get_logger().info(
                "首次旋转 360 度完成"
                f"（累计 {math.degrees(self.initial_accumulated_yaw):.1f} 度）；"
                f"下一点位={self.target_point_id}"
            )
            self.start_initial_wait()
            return

        self.publish_velocity(
            0.0,
            math.radians(self.initial_spin_speed_deg),
        )

    def do_initial_wait(self):
        self.publish_zero()
        if self.initial_stage_elapsed() < self.initial_wait_after_spin:
            return
        self.finish_initial_prepare()

    def initial_stage_elapsed(self):
        if self.initial_stage_started_at is None:
            return 0.0
        return (
            self.get_clock().now() - self.initial_stage_started_at
        ).nanoseconds / 1e9

    # ================================================================
    # Nav2 导航
    # ================================================================
    def start_navigation(self, point_id: str):
        point = self.points[point_id]

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
            f"下一点处理成功；下一点位={point_id}；"
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
    # 舵轮回正
    # ================================================================
    def start_steering_settle(self):
        self.publish_zero()
        self.settle_started_at = self.get_clock().now()
        self.set_state(
            PatrolState.SETTLING,
            f"持续零速度 {self.steering_settle_time:.1f} 秒，"
            "等待舵轮回正",
        )

    # ================================================================
    # ★ 对接：发请求 + 等待完成
    # ================================================================
    def start_docking(self):
        """向 docking_controller 发对接请求，进入 DOCKING 状态。"""
        self.publish_zero()
        self.docking_started_at = self.get_clock().now()

        req = String()
        req.data = self.target_point_id if self.target_point_id else "start"
        self.docking_request_pub.publish(req)

        self.set_state(
            PatrolState.DOCKING,
            f"请求 docking_controller 对接 {self.target_point_id} 号点",
        )
        self.get_logger().info(
            f"已发对接请求: {req.data}"
        )

    # ================================================================
    # 退出对接（纯直线后退，不涉及反光条）
    # ================================================================
    def start_relative_motion(self, new_state: PatrolState):
        self.publish_zero()

        self.motion_origin = self.get_odom_xy()
        self.motion_started_at = self.get_clock().now()

        if new_state == PatrolState.UNDOCKING:
            detail = (
                f"退出 {self.current_point_id} 号点"
            )
        else:
            detail = new_state.value

        self.set_state(new_state, detail)

    # ================================================================
    # 主循环
    # ================================================================
    def on_timer(self):
        # 定期用 map -> base_footprint 的实际坐标同步点位。
        self._sync_current_point_from_pose()

        if self.state == PatrolState.INITIAL_BACKUP:
            self.do_initial_backup()
            return

        if self.state == PatrolState.INITIAL_SPIN_360:
            self.do_initial_spin()
            return

        if self.state == PatrolState.INITIAL_WAIT:
            self.do_initial_wait()
            return

        # -------- SETTLING --------
        if self.state == PatrolState.SETTLING:
            self.publish_zero()

            elapsed = (
                self.get_clock().now() - self.settle_started_at
            ).nanoseconds / 1e9

            if elapsed >= self.steering_settle_time:
                self.settle_started_at = None
                self.get_logger().info(
                    "舵轮回正等待完成，请求对接"
                )
                self.start_docking()
            return

        # -------- ★ DOCKING：等 docking_controller 完成 --------
        if self.state == PatrolState.DOCKING:
            if self.docking_started_at is None:
                return
            elapsed = (
                self.get_clock().now() - self.docking_started_at
            ).nanoseconds / 1e9
            if elapsed > self.docking_timeout:
                self.publish_zero()
                self.fail(
                    f"对接超时，{self.docking_timeout:.1f}s 未收到完成信号"
                )
            return

        # -------- UNDOCKING：直线后退 --------
        if self.state != PatrolState.UNDOCKING:
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

        if distance >= self.undock_distance:
            self.finish_undocking()
            return

        self.publish_velocity(-self.undock_speed)

    # ================================================================
    # 当前位置与停靠状态同步
    # ================================================================
    def _sync_current_point_from_pose(self):
        """仅根据 map 中的二维位置识别最近点位，不检查航向角。"""
        now = self.get_clock().now()
        if self.last_docked_verify_at is not None:
            elapsed = (
                now - self.last_docked_verify_at
            ).nanoseconds / 1e9
            if elapsed < self.docked_verify_interval:
                return

        self.last_docked_verify_at = now

        current_xy = self.get_map_xy()
        if current_xy is None:
            return

        nearest_point_id = None
        nearest_distance = float("inf")
        for point_id, point in self.points.items():
            distance = math.hypot(
                current_xy[0] - point["x"],
                current_xy[1] - point["y"],
            )
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_point_id = point_id

        detected_point_id = (
            nearest_point_id
            if nearest_distance <= self.docked_position_tolerance
            else None
        )
        self.publish_located_point(
            detected_point_id,
            f"位置识别距离={nearest_distance:.3f} m",
        )

        # current_point_id 表示“已完成对接”。一旦实际位置离开该点，
        # 只清除对接记录；不会因为手动开到其他点附近就假定已对接。
        if (
            self.current_point_id is not None
            and detected_point_id != self.current_point_id
        ):
            old_point_id = self.current_point_id
            self.current_point_id = None
            self.get_logger().warn(
                f"机器人已离开 {old_point_id} 号对接点，"
                "清除已对接记录"
            )
            if self.state == PatrolState.DOCKED:
                self.set_state(PatrolState.IDLE, "位置偏离停靠点")

    # ================================================================
    # 完成
    # ================================================================
    def finish_docking(self):
        self.publish_zero()

        point_id = self.target_point_id
        self.current_point_id = point_id
        self.publish_located_point(
            point_id,
            "导航和对接完成",
            force=True,
        )

        self.motion_origin = None
        self.motion_started_at = None
        self.docking_started_at = None
        self.last_docked_verify_at = None

        self.set_state(
            PatrolState.DOCKED,
            f"已完成 {point_id} 号点对接",
        )

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
    def publish_velocity(
        self,
        linear_x: float,
        angular_z: float = 0.0,
    ):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.angular.z = float(angular_z)
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

    def publish_located_point(
        self,
        point_id,
        reason: str = "",
        force: bool = False,
    ):
        """发布根据二维坐标识别的所在点；空字符串表示无匹配点。"""
        if not force and point_id == self.located_point_id:
            return

        old_point_id = self.located_point_id
        self.located_point_id = point_id

        msg = String()
        msg.data = point_id or ""
        self.current_point_pub.publish(msg)

        self.get_logger().info(
            f"所在点位更新: {old_point_id or 'None'} -> "
            f"{point_id or 'None'}；原因={reason}"
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
        next_point_id = self.target_point_id or "None"
        detail = f"{reason}；下一点位={next_point_id}"
        self.set_state(PatrolState.FAILED, detail)
        self.get_logger().error(
            f"任务失败；下一点位={next_point_id}；{reason}"
        )

    # ================================================================
    # TF
    # ================================================================
    def get_map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None

        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (
                rotation.w * rotation.z
                + rotation.x * rotation.y
            ),
            1.0 - 2.0 * (
                rotation.y * rotation.y
                + rotation.z * rotation.z
            ),
        )

        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            yaw,
        )

    def get_map_xy(self):
        pose = self.get_map_pose()
        if pose is None:
            return None

        return pose[0], pose[1]

    @staticmethod
    def normalize_angle(angle: float):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

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
