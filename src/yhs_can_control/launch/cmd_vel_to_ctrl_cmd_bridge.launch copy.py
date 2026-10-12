from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gear',
            default_value='8',
            description='6=4T4D (linear.x + angular.z), 8=lateral (linear.x + linear.y)',
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='CtrlCmd publishing frequency in Hz (minimum 50)',
        ),
        DeclareLaunchArgument(
            'cmd_timeout',
            default_value='0.2',
            description='Publish zero velocity after this many seconds without cmd_vel',
        ),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_smoothed'),
        DeclareLaunchArgument('ctrl_cmd_topic', default_value='/ctrl_cmd'),
        Node(
            package='yhs_can_control',
            executable='cmd_vel_to_ctrl_cmd_bridge',
            name='cmd_vel_to_ctrl_cmd_bridge',
            output='screen',
            parameters=[{
                'gear': ParameterValue(
                    LaunchConfiguration('gear'), value_type=int
                ),
                'publish_rate': ParameterValue(
                    LaunchConfiguration('publish_rate'), value_type=float
                ),
                'cmd_timeout': ParameterValue(
                    LaunchConfiguration('cmd_timeout'), value_type=float
                ),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'ctrl_cmd_topic': LaunchConfiguration('ctrl_cmd_topic'),
            }],
        ),
    ])
