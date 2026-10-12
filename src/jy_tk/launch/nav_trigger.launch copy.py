from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        # ---------- 目标点 ----------
        DeclareLaunchArgument('goal_x',             default_value='-1'),
        DeclareLaunchArgument('goal_y',             default_value='13.0'),
        DeclareLaunchArgument('goal_yaw',           default_value='0.0'),

        # ---------- BACKUP ----------
        DeclareLaunchArgument('backup_dist',        default_value='0.3'),
        DeclareLaunchArgument('backup_speed',       default_value='0.15'),
        DeclareLaunchArgument('backup_timeout',     default_value='10.0'),

        # ---------- SPIN_360 ----------
        DeclareLaunchArgument('spin_360_wz_deg',    default_value='60.0'),
        DeclareLaunchArgument('spin_360_timeout',   default_value='30.0'),

        # ---------- DIAGONAL_MOVE ----------
        DeclareLaunchArgument('diagonal_dist',      default_value='0.5'),
        DeclareLaunchArgument('diagonal_speed',     default_value='0.15'),
        DeclareLaunchArgument('diagonal_angle_deg', default_value='-135.0'),
        DeclareLaunchArgument('diagonal_timeout',   default_value='15.0'),

        # ---------- ALIGN ----------
        DeclareLaunchArgument('align_tolerance_rad',default_value='0.10'),
        DeclareLaunchArgument('align_kp',           default_value='1.5'),
        DeclareLaunchArgument('align_max_wz_deg',   default_value='30.0'),
        DeclareLaunchArgument('align_timeout',      default_value='15.0'),

        # ---------- 控制 ----------
        DeclareLaunchArgument('cmd_vel_topic',      default_value='/cmd_vel_smoothed'),
    ]

    node = Node(
        package='jy_tk',
        executable='nav_trigger.py',
        name='nav_trigger',
        output='screen',
        parameters=[{
            'goal_x':              ParameterValue(LaunchConfiguration('goal_x'),              value_type=float),
            'goal_y':              ParameterValue(LaunchConfiguration('goal_y'),              value_type=float),
            'goal_yaw':            ParameterValue(LaunchConfiguration('goal_yaw'),            value_type=float),
            'backup_dist':         ParameterValue(LaunchConfiguration('backup_dist'),         value_type=float),
            'backup_speed':        ParameterValue(LaunchConfiguration('backup_speed'),        value_type=float),
            'backup_timeout':      ParameterValue(LaunchConfiguration('backup_timeout'),      value_type=float),
            'spin_360_wz_deg':     ParameterValue(LaunchConfiguration('spin_360_wz_deg'),     value_type=float),
            'spin_360_timeout':    ParameterValue(LaunchConfiguration('spin_360_timeout'),    value_type=float),
            'diagonal_dist':       ParameterValue(LaunchConfiguration('diagonal_dist'),       value_type=float),
            'diagonal_speed':      ParameterValue(LaunchConfiguration('diagonal_speed'),      value_type=float),
            'diagonal_angle_deg':  ParameterValue(LaunchConfiguration('diagonal_angle_deg'),  value_type=float),
            'diagonal_timeout':    ParameterValue(LaunchConfiguration('diagonal_timeout'),    value_type=float),
            'align_tolerance_rad': ParameterValue(LaunchConfiguration('align_tolerance_rad'), value_type=float),
            'align_kp':            ParameterValue(LaunchConfiguration('align_kp'),            value_type=float),
            'align_max_wz_deg':    ParameterValue(LaunchConfiguration('align_max_wz_deg'),    value_type=float),
            'align_timeout':       ParameterValue(LaunchConfiguration('align_timeout'),       value_type=float),
            'cmd_vel_topic':       LaunchConfiguration('cmd_vel_topic'),
        }],
    )

    return LaunchDescription(args + [node])