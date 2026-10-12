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
            
            # LidarType 网口雷达型号
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
            #   LT_TW360,              :21
            #   LT_Scope128S,          :22
            #   LT_C2503,              :23
            #   LT_Scope_G,            :25
            #   LT_TW360S_24,          :26
            
            # LidarType 串口雷达型号
            #   LT_TW360S_24_serialPort_A2,      :29
            #   LT_TW360S_16_serialPort_A2,      :30
            #   LT_TW360S_24_serialPort_B,       :31
            parameters=[{
                'LidarType': 21, # 雷达型号
                'ConnectType': 'on-line', # 连接模式:仅支持"on-line"、"off-line"两种模式配置
                'PcapFilePath': '/home/tanway/lidar_data/tanway.pcap', # 回放模式下配置: PCAP文件绝对路径
                'topic': "/tanwaylidar_pointcloud",
                'imu_topic': "/tanwaylidar_imu",
                'LocalHost': "192.168.31.118",
                'LocalPointloudPort': 5600,
                'LocalDIFPort': 5700,
                'LocalIMUPort': 5900,
                'LocalPointloudPort_2': 6602,
                'LocalDIFPort_2': 6702,
                'LidarHost': "192.168.31.51",
                'AlgoTablePath': os.path.join(get_package_share_directory(pkg_name), 'config', "algo_table.json"),
                'SerialRingTypeConfigPath': os.path.join(get_package_share_directory(pkg_name), 'config', "serial_ring_type_config.json"),

                # 仅串口雷达型号生效（S24 示例）
                'SerialPort': "/dev/ttyACM0",
                'SerialControlPort': "/dev/ttyACM1",
                'SerialControlBaudRate': 2000000,
                'SerialBaudRate': 6000000,
                'IsDualPort': False,
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
                'UseLidarClock': False,
                'CycleCountFrameSplit': False,

                # 时间分帧模式True、扫描分帧模式False
                'UseTimeWindow': False,

                # udp ros msg发布
	            'UseUdpROSmsg': False,

                # 以秒为时间戳单位发布点云
	            'UsePointTimeSec': True,
	            
                # 点云下采样倍数设置
	            'VerticalSampleInterval': 1,
                
                # IMU线性加速度输出单位设置 true = m/s²，false = g
                'ImuAccelToMs2': True,

		        'DistanceMin' : 0.0,
    	        'DistanceMax' : 200.0
                
                }],
            output='log'
        ),
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', [os.path.join(get_package_share_directory(pkg_name), 'rviz', "show.rviz")]]
        # ),
    ])
