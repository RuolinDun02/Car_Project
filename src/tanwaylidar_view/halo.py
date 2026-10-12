from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'tanwaylidar_view'
    return LaunchDescription([
        Node(
            package='tanwaylidar_view',
            namespace='tanwaylidar_view',
            executable='TanwayLidar_MainNode',
            name='TanwayLidar_MainNode',
            
            parameters=[{
                'LidarType': 21,  # 这里修复：加了逗号
                'ConnectType': 'on-line',
                'PcapFilePath': '/home/tanway/lidar_data/tanway.pcap',
                'topic': "/tanwaylidar_pointcloud",
                'imu_topic': "/tanwaylidar_imu",
                'LocalHost': "192.168.111.204",
                'LocalPointloudPort': 5600,
                'LocalDIFPort': 5700,
                'LocalIMUPort': 5900,
                'LocalPointloudPort_2': 6602,
                'LocalDIFPort_2': 6702,
                'LidarHost': "192.168.111.51",
                'AlgoTablePath': os.path.join(get_package_share_directory(pkg_name), 'config', "algo_table.json"),

                'CoreHorAngleOffsetL': 0.0,
                'CoreHorAngleOffsetR': 0.0,
                'CoreVerAngleOffsetL': 0.0,
                'CoreVerAngleOffsetR': 0.0,
                
                'MirrorVerAngleOffsetA': 0.0,
                'MirrorVerAngleOffsetB': 0.0,
                'MirrorVerAngleOffsetC': 0.0,
                'MirrorHorAngleOffsetA': 0.0,
                'MirrorHorAngleOffsetB': 0.0,
                'MirrorHorAngleOffsetC': 0.0,

                # ========== FAST-LIO 必须正确的参数 ==========
                'FrameTimeStampType': "first_point",
                'UseLidarClock': False,
                'DistanceMin' : 0.1,
                # ==============================================

                'CycleCountFrameSplit': False,
                'UseTimeWindow': False,
                'UseUdpROSmsg': False,
                'UsePointTimeSec': True,
            
                'VerticalSampleInterval': 1,
                'ImuAccelToMs2': True,
                'DistanceMax' : 200.0
            }],
            output='screen'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory(pkg_name), 'rviz', "show.rviz")]
        ),
    ])

