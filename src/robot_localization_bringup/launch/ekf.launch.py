import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_localization_bringup')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        # ---- 重映射：把 EKF 输出的 /odometry/filtered 改名为 /odom ----
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
    )

    return LaunchDescription([ekf_node])