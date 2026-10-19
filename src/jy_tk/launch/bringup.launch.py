"""
jy_tk 主启动文件。

用法：
  仿真：ros2 launch jy_tk bringup.launch.py use_sim:=true
  真机：ros2 launch jy_tk bringup.launch.py use_sim:=false

切换逻辑：
  use_sim=true  → 启动 Gazebo + 机器人模型（自带电机、雷达仿真）
  use_sim=false → 启动真机硬件层（电机驱动 + 雷达驱动 + 看门狗）
  公共层        → robot_state_publisher + 3D→2D 点云转换
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_pcl_to_scan(context):
    """
    根据 use_sim 参数，动态选择输入点云话题：
      仿真 → /rslidar_points（Gazebo velodyne 插件输出）
      真机 → /tanwaylidar_pointcloud（Tanway Halo 输出）
    输出统一为 /scan，供 slam_toolbox / RViz 使用。
    """
    use_sim_value = LaunchConfiguration('use_sim').perform(context)

    if use_sim_value.lower() == 'true':
        cloud_topic = '/rslidar_points'
    else:
        cloud_topic = '/tanwaylidar_pointcloud'

    pcl_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', cloud_topic),
            ('scan',     '/scan'),
        ],
        parameters=[{
            'target_frame': 'lidar_link',
            'transform_tolerance': 10.0,
            'min_height':  -0.3,
            'max_height':   1.7,
            'angle_min':  -3.14159,
            'angle_max':   3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 30.0,
            'use_inf': True,
            'use_sim_time': LaunchConfiguration('use_sim'),
        }],

    )
    return [pcl_to_scan]


def generate_launch_description():
    pkg_share = get_package_share_directory('jy_tk')

    # ============================================================
    # Launch 参数
    # ============================================================
    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='true=仿真, false=真机'
    )
    use_motor_arg = DeclareLaunchArgument(
        'use_motor', default_value='true',
        description='true=启动电机驱动, false=只用雷达'
    )
    # ★ 新增：是否启动电机看门狗
    use_motor_watchdog_arg = DeclareLaunchArgument(
        'use_motor_watchdog', default_value='false',
        description='true=启动电机看门狗（失能自动恢复）, false=不启动'
    )

    use_sim = LaunchConfiguration('use_sim')
    use_motor = LaunchConfiguration('use_motor')
    use_motor_watchdog = LaunchConfiguration('use_motor_watchdog')

    # ============================================================
    # URDF
    # ============================================================
    urdf_file = os.path.join(pkg_share, 'urdf', 'jy_tk.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # ============================================================
    # 公共层：robot_state_publisher
    # ============================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim,
        }],
    )

    # ============================================================
    # 公共层：RViz（已注释）
    # ============================================================
    # rviz_config = os.path.join(pkg_share, 'rviz', 'jy_tk.rviz')
    # rviz = Node(...)

    # ============================================================
    # 公共层：3D 点云 → 2D LaserScan
    # ============================================================
    pcl_to_scan_action = OpaqueFunction(function=_make_pcl_to_scan)

    # ============================================================
    # 仿真硬件层
    # ============================================================
    sim_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'include', 'sim_hardware.launch.py')
        ),
        condition=IfCondition(use_sim),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # ============================================================
    # 真机硬件层
    # ============================================================
    real_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'include', 'real_hardware.launch.py')
        ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            'use_sim_time': 'false',
            'use_motor': use_motor,
        }.items(),
    )

    # ============================================================
    # ★ 新增：电机看门狗
    # ------------------------------------------------------------
    # 只在 真机 + 用电机 + 启用看门狗 时启动
    #
    # 作用：监控 /chassis_info_fb 里的电机使能状态，
    #       检测到失能（压电线/急停等触发 VCU 保护）时
    #       自动发 /motor_cmd 恢复使能。
    #
    # 禁用方式（运行时）：
    #   ros2 topic pub --once /motor_watchdog/enable \
    #     std_msgs/Bool '{data: false}'
    # ============================================================
    motor_watchdog = Node(
        package='jy_tk',
        executable='motor_watchdog.py',
        name='motor_watchdog',
        output='screen',
        # 条件：仿真时不启动（仿真没有真实电机）
        #       use_motor=false 时不启动（不用电机就不用看门狗）
        #       use_motor_watchdog=false 时不启动（用户显式关闭）
        condition=IfCondition(use_motor_watchdog),
        parameters=[{
            'check_interval':   0.5,
            'retry_interval':   2.0,
            'max_retry':        10,
            'publish_times':    5,
        }],
    )

    return LaunchDescription([
        use_sim_arg,
        use_motor_arg,
        use_motor_watchdog_arg,   # ★ 新增
        robot_state_publisher,
        sim_hardware,
        real_hardware,
        pcl_to_scan_action,
        motor_watchdog,           # ★ 新增
        # rviz,
    ])