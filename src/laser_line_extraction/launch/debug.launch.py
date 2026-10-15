from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_line_extraction',
            executable='line_extraction_node',
            name='line_extractor',
            output='screen',
            parameters=[{
                'scan_topic': 'front_lidar/scan',
                'frequency': 50.0,
                'publish_markers': True,
            }]
        )
    ])
