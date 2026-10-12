"""
真机硬件层：电机驱动 + 雷达驱动 + EKF + 必要的静态 TF。

use_motor=true  → 启动电机驱动 + EKF
                  EKF 发 odom → base_footprint TF
use_motor=false → 只启动雷达
                  静态 TF 发 odom → base_footprint（恒等变换）

TF 树最终形态：
  map                      ← slam_toolbox 发
   └─ odom                 ← slam_toolbox 发
       └─ base_footprint   ← EKF 发（use_motor=true）
                          或 本文件的静态 TF（use_motor=false）
           └─ base_link    ← URDF 发（robot_state_publisher）
               └─ lidar_link
                   └─ TanwayTP  ← 本文件的静态 TF（-90° 旋转）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    jy_tk_share = get_package_share_directory('jy_tk')
    lidar_share = get_package_share_directory('tanwaylidar_view')
    rl_share    = get_package_share_directory('robot_localization_bringup')

    # ---------- 参数 ----------
    use_motor_arg = DeclareLaunchArgument(
        'use_motor', default_value='true',
        description='true=启动电机驱动+EKF, false=只用雷达（静态TF）'
    )
    use_motor = LaunchConfiguration('use_motor')

    # ---------- 1) 电机驱动（条件启动）----------
    motor_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yhs_can_control'),
                'launch', 'yhs_can_control.launch.py'
            )
        ),
        condition=IfCondition(use_motor),
    )

    # ---------- 2) 雷达驱动（始终启动）----------
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_share, 'launch', 'tanwaylidar.py')
        ),
    )

    # ---------- 3) EKF 融合（只在 use_motor=true 时启动）----------
    # 订阅 /odom_base + /tanwaylidar_imu
    # 输出 /odom（重映射后）+ odom → base_footprint TF
    # 注意：ekf.yaml 里 base_link_frame 必须是 base_footprint
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rl_share, 'launch', 'ekf.launch.py')
        ),
        condition=IfCondition(use_motor),
    )

    # ---------- 4) 静态 TF：odom → base_footprint ----------
    # 只在 use_motor=false 时启动，避免和 EKF 的 TF 冲突。
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_base_footprint',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'odom',
            '--child-frame-id', 'base_footprint',
        ],
        condition=UnlessCondition(use_motor),
    )

    # ---------- 5) 静态 TF：lidar_link → TanwayTP ----------
    # 雷达点云 frame 是 TanwayTP，URDF 里雷达 frame 是 lidar_link，
    # 用一个恒等变换把两者关联，TF 树才连通。
    # 雷达顺时针旋转 90° 安装，用 -90° 的 Z 轴旋转补偿。
    lidar_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_link_to_tanway',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0',
            '--qz', '-0.7071068',    # sin(-45°)
            '--qw', '0.7071068',     # cos(-45°)
            '--frame-id', 'lidar_link',
            '--child-frame-id', 'TanwayTP',
        ],
    )

    # ---------- 6) cmd_vel → ctrl_cmd 桥接 ----------
    # 订阅 /cmd_vel_smoothed（velocity_smoother 输出）
    # 发布 /ctrl_cmd 给 yhs_can_control_node
    # 只在 use_motor=true 时启动（需要底盘驱动在运行）
    cmd_vel_bridge = Node(
        package='yhs_can_control',
        executable='cmd_vel_to_ctrl_cmd_bridge',
        name='cmd_vel_to_ctrl_cmd_bridge',
        output='screen',
        condition=IfCondition(use_motor),
        parameters=[{
            'gear': 6,
            'publish_rate': 50.0,
            'cmd_timeout': 0.2,
            'cmd_vel_topic': '/cmd_vel_smoothed',
            'ctrl_cmd_topic': '/ctrl_cmd',
        }],
    )

    return LaunchDescription([
        use_motor_arg,
        motor_node,
        lidar_launch,
        ekf_launch,
        static_odom_tf,
        lidar_frame_tf,
        cmd_vel_bridge,
    ])