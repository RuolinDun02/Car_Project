"""
仿真硬件层：启动 Gazebo + 机器人（含 skid_steer_drive、velodyne 插件）。

真机不启动这一层。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jy_tk')
    world_file = os.path.join(pkg_share, 'worlds', 'nav.world')  # 若有世界文件
    urdf_file  = os.path.join(pkg_share, 'urdf', 'jy_tk_gazebo.urdf')

    # 如果没有 worlds/nav.world，可换成 mickrobot_description 的
    if not os.path.exists(world_file):
        try:
            mick_share = get_package_share_directory('mickrobot_description')
            world_file = os.path.join(mick_share, 'worlds', 'nav.world')
        except Exception:
            world_file = ''

    # 启动 Gazebo 世界
    if world_file:
        gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen',
        )
    else:
        gazebo = ExecuteProcess(
            cmd=['gazebo', '--verbose',
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen',
        )

    # 延迟 spawn，避免服务未就绪
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'jy_tk', '-file', urdf_file,
                   '-timeout', '120.0'],
        output='screen',
    )
    delayed_spawn = TimerAction(period=8.0, actions=[spawn])

    return LaunchDescription([
        gazebo,
        delayed_spawn,
    ])