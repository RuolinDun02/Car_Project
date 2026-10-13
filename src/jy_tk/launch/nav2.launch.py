"""
Nav2 导航 launch：硬件层 + 静态地图 + AMCL + 规划 + 控制。
（桥接节点已放到 real_hardware.launch.py 里）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('jy_tk')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # ---------- 参数 ----------
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_motor = LaunchConfiguration('use_motor', default='true')
    map_yaml = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_share, 'maps', 'my_map3.yaml'),
    )
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
    )

    # ---------- 1) 硬件层（雷达 + 电机 + EKF + 桥接 + /scan）----------
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'use_sim': use_sim_time,
            'use_motor': use_motor,
        }.items(),
    )

    # ---------- 2) Nav2 官方 bringup ----------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
            'use_robot_state_pub': 'False',        # ★ 加这一行,让默认bringup发
            'slam': 'False',
        }.items(),
    )

    # Start the hardware, static TF and cloud-to-scan node first.  The
    # converter uses a lazy cloud subscription; bringing it up before Nav2
    # creates its /scan subscribers avoids racing lifecycle transitions.
    delayed_nav2_bringup = TimerAction(
        period=5.0,
        actions=[nav2_bringup],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='仿真用 true，真机用 false',
        ),
        DeclareLaunchArgument(
            'use_motor', default_value='true',
            description='true=启动电机驱动+EKF, false=只用雷达',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'my_map3.yaml'),
            description='地图 yaml 路径',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Nav2 参数文件',
        ),
        bringup_launch,
        delayed_nav2_bringup,
    ])
