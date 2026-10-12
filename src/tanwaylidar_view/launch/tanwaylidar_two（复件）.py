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
            
            # LidarType 雷达型号
            #   LT_Tensor16            :0
            #   LT_Tensor32            :1
            #   LT_Scope192            :2
            #   LT_Duetto              :3
            #   LT_TempoA1             :4
            #   LT_TempoA2             :5
            #   LT_ScopeMini           :6
            #   LT_TempoA3             :7
            #   LT_TempoA4             :8
            #   LT_Tensor48            :9
            #   LT_Tensor48_Depth      :10
            #   LT_Scope256            :11
            #   LT_Scope256_Depth      :12
            #   LT_FocusB1             :13
            #   LT_Scope256_SmallBlind :14
            #   LT_FocusB2_B3_MP,      :15
            #   LT_Scope128H,          :16
            #   LT_Scope128,           :17
            #   LT_Scope128F,          :18
            #   LT_FocusB2_64,         :19
            #   LT_FocusT,             :20
            #   LT_TW360               :21
            parameters=[{
                'LidarType': 21, # 雷达型号
                'ConnectType': 'on-line', # 连接模式:仅支持"on-line"、"off-line"两种模式配置
                'PcapFilePath': '/home/tanway/lidar_data/tanway.pcap', # 回放模式下配置: PCAP文件绝对路径
                'topic': "/tanwaylidar_pointcloud",
                'imu_topic': "/tanwaylidar_imu",
                'LocalHost': "192.168.110.204",
                'LocalPointloudPort': 5600,
                'LocalDIFPort': 5700,
                'LocalIMUPort': 5900,
                'LidarHost': "192.168.110.51",
                'AlgoTablePath': os.path.join(get_package_share_directory(pkg_name), 'config', "algo_table.json"),

                
                # Set the lidar core horizontal angle offset(Only support Scope series and Duetto). 
                'CoreHorAngleOffsetL': 0.0,
                'CoreHorAngleOffsetR': 0.0,

                # Set the lidar core vertical angle offset(Only support Scope series and Duetto).
                'CoreVerAngleOffsetL': 0.0,
                'CoreVerAngleOffsetR': 0.0,
                
                # Set the lidar mirror abc vertical angle offset.
                'MirrorVerAngleOffsetA': 0.0,
                'MirrorVerAngleOffsetB': 0.0,
                'MirrorVerAngleOffsetC': 0.0,

                # Set the lidar mirror abc horizontal angle offset.
                'MirrorHorAngleOffsetA': 0.0,
                'MirrorHorAngleOffsetB': 0.0,
                'MirrorHorAngleOffsetC': 0.0,

                # 帧时间戳
                'FrameTimeStampType': "last_point",
                'UseLidarClock': True,
                'CycleCountFrameSplit': False,

                # 时间分帧模式True、扫描分帧模式False
                'UseTimeWindow': False
                }],
            output='screen'
        ),
        
        Node(
            package='tanwaylidar_view',
            namespace='tanwaylidar_view',
            executable='TanwayLidar_MainNode',
            name='TanwayLidar_MainNode1',           #两颗雷达修改1
            
            # LidarType 雷达型号
            #   LT_Tensor16            :0
            #   LT_Tensor32            :1
            #   LT_Scope192            :2
            #   LT_Duetto              :3
            #   LT_TempoA1             :4
            #   LT_TempoA2             :5
            #   LT_ScopeMini           :6
            #   LT_TempoA3             :7
            #   LT_TempoA4             :8
            #   LT_Tensor48            :9
            #   LT_Tensor48_Depth      :10
            #   LT_Scope256            :11
            #   LT_Scope256_Depth      :12
            #   LT_FocusB1             :13
            #   LT_Scope256_SmallBlind :14
            #   LT_FocusB2_B3_MP,      :15
            #   LT_Scope128H,          :16
            #   LT_Scope128,           :17
            #   LT_Scope128F,          :18
            #   LT_FocusB2_64,         :19
            #   LT_FocusT,             :20
            #   LT_TW360               :21
            parameters=[{
                'LidarType': 21, # 雷达型号
                'ConnectType': 'on-line', # 连接模式:仅支持"on-line"、"off-line"两种模式配置
                'PcapFilePath': '/home/tanway/lidar_data/tanway.pcap', # 回放模式下配置: PCAP文件绝对路径
                'topic': "/tanwaylidar_pointcloud1",#两颗雷达修改2
                'imu_topic': "/tanwaylidar_imu1",#两颗雷达修改3
                'LocalHost': "192.168.110.204",
                'LocalPointloudPort': 5601,#两颗雷达修改4
                'LocalDIFPort': 5701,#两颗雷达修改5
                'LocalIMUPort': 5901,#两颗雷达修改6
                'LidarHost': "192.168.110.52",#两颗雷达修改7
                'AlgoTablePath': os.path.join(get_package_share_directory(pkg_name), 'config', "algo_table.json"),

                
                # Set the lidar core horizontal angle offset(Only support Scope series and Duetto). 
                'CoreHorAngleOffsetL': 0.0,
                'CoreHorAngleOffsetR': 0.0,

                # Set the lidar core vertical angle offset(Only support Scope series and Duetto).
                'CoreVerAngleOffsetL': 0.0,
                'CoreVerAngleOffsetR': 0.0,
                
                # Set the lidar mirror abc vertical angle offset.
                'MirrorVerAngleOffsetA': 0.0,
                'MirrorVerAngleOffsetB': 0.0,
                'MirrorVerAngleOffsetC': 0.0,

                # Set the lidar mirror abc horizontal angle offset.
                'MirrorHorAngleOffsetA': 0.0,
                'MirrorHorAngleOffsetB': 0.0,
                'MirrorHorAngleOffsetC': 0.0,

                # 帧时间戳
                'FrameTimeStampType': "last_point",
                'UseLidarClock': True,
                'CycleCountFrameSplit': False,

                # 时间分帧模式True、扫描分帧模式False
                'UseTimeWindow': False
                }],
            output='screen'
        ),
        
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(get_package_share_directory(pkg_name), 'rviz', "show.rviz")]]
        ),
    ])
