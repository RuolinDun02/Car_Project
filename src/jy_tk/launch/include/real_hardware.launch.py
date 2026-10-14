"""
真机硬件层：电机驱动 + 雷达驱动 + EKF + 必要的静态 TF。

use_motor=true  → 启动电机驱动 + EKF
                  EKF 发 odom → base_footprint TF
                  同时启动：反光条提取 + 对接控制器
use_motor=false → 只启动雷达
                  静态 TF 发 odom → base_footprint（恒等变换）
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
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rl_share, 'launch', 'ekf.launch.py')
        ),
        condition=IfCondition(use_motor),
    )

    # ---------- 4) 静态 TF：odom → base_footprint ----------
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
    lidar_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_link_to_tanway',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0',
            '--qz', '-0.7071068',
            '--qw', '0.7071068',
            '--frame-id', 'lidar_link',
            '--child-frame-id', 'TanwayTP',
        ],
    )

    # ---------- 6) cmd_vel → ctrl_cmd 桥接 ----------
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

    # ---------- 7) ★ 反光条直线提取 ----------
    tanway_line_extractor = Node(
        package='tanway_line_extractor',
        executable='tanway_line_extractor_node',
        name='tanway_line_extractor',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(use_motor),
        parameters=[{
            'cloud_topic': '/tanwaylidar_pointcloud',
            'lines_topic': '/tanway_lines',
            'markers_topic': '/tanway_line_markers',
            'frame_id': 'lidar_link',
            'extraction_mode': '3d_cluster',
            'reflective_threshold': 100.0,
            'cluster_eps': 0.08,
            'cluster_min_pts': 15,
            'linearity_threshold': 8.0,
            'reflective_line_min_length': 0.20,
            'reflective_line_max_length': 3.00,
            'reflective_z_min': -0.50,
            'reflective_z_max': 1.50,
            'processing_rate': 10.0,
            'cloud_queue_size': 3,
            'publish_refl_cloud': True,
            'refl_cloud_topic': '/tanway_reflective_points',
        }],
    )

    # ---------- 8) ★ 对接控制器 ----------
    docking_controller = Node(
        package='jy_tk',
        executable='docking_controller.py',
        name='docking_controller',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(use_motor),
        parameters=[{
            'lines_topic': '/tanway_lines',
            'cmd_vel_topic': '/cmd_vel_smoothed',
            'request_topic': '/docking/request',
            'complete_topic': '/docking/complete',
            'state_topic': '/docking/state',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'min_line_length': 0.15,
            'max_line_length': 1.50,
            'min_target_x': 0.20,
            'max_target_x': 3.00,
            'max_target_y': 0.80,
            'target_distance': 0.25,
            'distance_tolerance': 0.05,
            'lateral_tolerance': 0.03,
            'kp_lateral': 1.5,
            'kp_yaw': 2.0,
            'vx_max': 0.10,
            'vx_min': 0.03,
            'wz_max': 0.40,
            'wait_lines_timeout': 3.0,
            'line_lost_timeout': 1.0,
            'align_overall_timeout': 30.0,
            'fallback_dock_speed': 0.08,
            'fallback_dock_distance': 0.7,
            'fallback_timeout': 20.0,
            'control_rate': 20.0,
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
        tanway_line_extractor,   # ★ 新增
        docking_controller,      # ★ 新增
    ])
