"""
tanway_line_extractor 启动文件。

功能：
  - 启动 tanway_line_extractor 节点（点云分割成线段）
  - 可选：发布 lidar_link → TanwayTP 的静态 TF

为什么加 static_transform_publisher：
  雷达点云的 frame 是 TanwayTP，而 markers 的 frame 是 lidar_link。
  数值上两者相同（恒等变换），但 RViz 需要 TF 链才能跨 frame 显示。
  这个静态 TF 补齐了 lidar_link → TanwayTP，让 RViz 可以用 lidar_link
  作为 Fixed Frame 同时显示点云和分割直线。

注意：
  如果 bringup.launch.py 里已经发过这个 TF，请设 use_static_tf:=false，
  避免两个 publisher 发同一个 transform 导致抖动。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tanway_line_extractor')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # ---------- 参数 ----------
    # use_static_tf:
    #   true  = 本 launch 发布 lidar_link → TanwayTP 的静态 TF
    #   false = 不发布（bringup 里已发过，或 TF 树里已有）
    use_static_tf_arg = DeclareLaunchArgument(
        'use_static_tf',
        default_value='true',
        description='是否发布 lidar_link → TanwayTP 的静态 TF'
    )
    use_static_tf = LaunchConfiguration('use_static_tf')

    # ---------- 1) 线段分割节点 ----------
    extractor_node = Node(
        package='tanway_line_extractor',
        executable='tanway_line_extractor_node',
        name='tanway_line_extractor',
        output='screen',
        parameters=[params_file],
    )

    # ---------- 2) 静态 TF: lidar_link → TanwayTP ----------
    # 恒等变换，只补齐 TF 链，不改变坐标
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_link_to_tanway_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '3.141592653', '--pitch', '0.0', '--yaw', '0',
            '--frame-id', 'lidar_link',
            '--child-frame-id', 'TanwayTP',
        ],
        condition=IfCondition(use_static_tf),
    )

    return LaunchDescription([
        use_static_tf_arg,
        extractor_node,
        static_tf,
    ])