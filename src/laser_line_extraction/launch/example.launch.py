from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_line_extraction',
            executable='line_extraction_node',
            name='line_extractor',
            parameters=[{
                'frequency': 30.0,
                'use_sim_time': True,
                'frame_id': 'lidar_link',
                'scan_topic': 'scan',
                'publish_markers': True,
                'bearing_std_dev': 1e-5,
                'range_std_dev': 0.012,
                'least_sq_angle_thresh': 0.0001,
                'least_sq_radius_thresh': 0.0001,
                'max_line_gap': 0.05,
                'min_line_length': 0.25,
                'min_range': 0.1,
                'max_range': 1.50,
                'min_split_dist': 0.04,
                'outlier_dist': 0.06,
                'min_line_points': 8,
            }]
        )
    ])
