from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        # ---------- 自动选档 ----------
        DeclareLaunchArgument(
            'auto_gear',
            default_value='true',
            description='true=根据 cmd_vel 自动切 gear 6/8, false=固定用 gear 参数',
        ),
        DeclareLaunchArgument(
            'gear',
            default_value='6',
            description='auto_gear=false 时的固定档位：6=4T4D, 8=横移',
        ),
        DeclareLaunchArgument(
            'lateral_threshold',
            default_value='0.02',
            description='判定"有横移需求"的 |linear.y| 阈值（m/s）',
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='CtrlCmd 发布频率（Hz，最低 50）',
        ),
        DeclareLaunchArgument(
            'cmd_timeout',
            default_value='0.2',
            description='多久没收到 cmd_vel 就发零速度（秒）',
        ),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_smoothed'),
        DeclareLaunchArgument('ctrl_cmd_topic', default_value='/ctrl_cmd'),

        Node(
            package='yhs_can_control',
            executable='cmd_vel_to_ctrl_cmd_bridge',
            name='cmd_vel_to_ctrl_cmd_bridge',
            output='screen',
            parameters=[{
                'auto_gear':         ParameterValue(LaunchConfiguration('auto_gear'),         value_type=bool),
                'gear':              ParameterValue(LaunchConfiguration('gear'),              value_type=int),
                'lateral_threshold': ParameterValue(LaunchConfiguration('lateral_threshold'), value_type=float),
                'publish_rate':      ParameterValue(LaunchConfiguration('publish_rate'),      value_type=float),
                'cmd_timeout':       ParameterValue(LaunchConfiguration('cmd_timeout'),       value_type=float),
                'cmd_vel_topic':     LaunchConfiguration('cmd_vel_topic'),
                'ctrl_cmd_topic':    LaunchConfiguration('ctrl_cmd_topic'),
            }],
        ),
    ])