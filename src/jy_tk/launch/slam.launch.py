import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('jy_tk')
    slam_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    # ---------- 参数 ----------
    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='true=仿真, false=真机'
    )
    use_motor_arg = DeclareLaunchArgument(
        'use_motor', default_value='true',
        description='true=启动电机驱动, false=只用雷达'
    )
    use_sim = LaunchConfiguration('use_sim')
    use_motor = LaunchConfiguration('use_motor')

    # ---------- 1) 先启动 bringup（硬件 + TF + /scan）----------
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'use_sim': use_sim,
            'use_motor': use_motor,
        }.items(),
    )

    # ---------- 2) 再启动 slam_toolbox ----------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim,
            'slam_params_file': slam_params,
        }.items(),
    )

    return LaunchDescription([
        use_sim_arg,
        use_motor_arg,
        bringup_launch,
        slam_launch,
    ])