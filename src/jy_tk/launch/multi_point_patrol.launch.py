import os

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("jy_tk")

    params_file = os.path.join(
        package_share,
        "config",
        "patrol_points.yaml",
    )

    patrol_node = Node(
        package="jy_tk",
        executable="multi_point_patrol.py",
        name="multi_point_patrol",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        patrol_node,
    ])