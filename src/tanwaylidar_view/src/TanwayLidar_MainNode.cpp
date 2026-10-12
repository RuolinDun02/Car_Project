/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 15-03-2022
 *  Author: LN

 *  Node for Tanway 3D LIDARs
**************************************************/

#if (defined(USE_FOR_ROS) || defined(USE_FOR_ROS2))

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

#endif

#ifdef USE_FOR_ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include "tanwaylidar_view/DIFString.h"
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>


#elif USE_FOR_ROS2
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <pcl_conversions/pcl_conversions.h>

#elif DISABLE_PCL_ROS
#include <sensor_msgs/point_cloud2_iterator.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include "tanwaylidar_view/DIFString.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#endif

#include "LaunchConfig.h"
#include "twlog.h"

#include <thread>

#include <string>

#if (defined(USE_FOR_ROS) || defined(DISABLE_PCL_ROS))
ros::Publisher rosPublisher;
ros::Publisher rosIMUPublisher;
ros::Publisher rosDIFPublisher;
#elif defined(USE_FOR_ROS2)
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rosPublisher;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr rosIMUPublisher;
#endif

LaunchConfig launchConfig;

class LidarObserver : public ILidarObserver
{
public:
	virtual void OnPointCloud(const LidarInfo &lidarInfo, const UserPointCloud &pointCloud)
	{
#ifdef USE_FOR_ROS
		// to ros point cloud
		sensor_msgs::PointCloud2 rosPointCloud;
		pcl::toROSMsg(pointCloud, rosPointCloud); // convert between PCL and ROS datatypes
		rosPublisher.publish(rosPointCloud);	  // Publish point cloud		
#elif defined(USE_FOR_ROS2)
		sensor_msgs::msg::PointCloud2 rosPointCloud;
		pcl::toROSMsg(pointCloud, rosPointCloud); // convert between PCL and ROS datatypes
		rosPublisher->publish(rosPointCloud);	  // Publish point cloud
#elif DISABLE_PCL_ROS
		// to ros point cloud
		sensor_msgs::PointCloud2 rosPointCloud;
		convertToPointCloud2(pointCloud,rosPointCloud,lidarInfo.lidarType);  //convert to ROS datatypes
		rosPublisher.publish(rosPointCloud);	  // Publish point cloud
#endif
	}
	virtual void OnIMU(const LidarInfo &lidarInfo, const IMUData &imu)
	{
/*
 *Avoid directly operating the UI in the callback function.
 */
#if defined(USE_FOR_ROS) || defined(DISABLE_PCL_ROS)
		sensor_msgs::Imu imu_data;
#elif defined(USE_FOR_ROS2)
		sensor_msgs::msg::Imu imu_data;
#endif

		uint64_t sec = imu.stamp / 1000000;
		uint64_t nsec = (imu.stamp  % 1000000) * 1000;
		imu_data.header.frame_id = imu.frame_id;

		imu_data.linear_acceleration.x = imu.linear_acceleration[0];
		imu_data.linear_acceleration.y = imu.linear_acceleration[1];
		imu_data.linear_acceleration.z = imu.linear_acceleration[2];

		imu_data.angular_velocity.x = imu.angular_velocity[0];
		imu_data.angular_velocity.y = imu.angular_velocity[1];
		imu_data.angular_velocity.z = imu.angular_velocity[2];

		std::cout << "IMU data callback:" << std::endl
			<< "  Angular velocity [rad/s]: "
			<< "X: " << std::fixed << std::setprecision(6)
			<< imu.angular_velocity[0] << ", Y: " << std::fixed
			<< std::setprecision(6) << imu.angular_velocity[1]
			<< ", Z: " << std::fixed << std::setprecision(6)
			<< imu.angular_velocity[2] << std::endl
			<< "  Linear acceleration [g]: "
			<< "X: " << std::fixed << std::setprecision(6)
			<< imu.linear_acceleration[0] << ", Y: " << std::fixed
			<< std::setprecision(6) << imu.linear_acceleration[1]
			<< ", Z: " << std::fixed << std::setprecision(6)
			<< imu.linear_acceleration[2] << std::endl;

#if defined(USE_FOR_ROS) || defined(DISABLE_PCL_ROS)
		imu_data.header.stamp = ros::Time(sec, nsec);
		rosIMUPublisher.publish(imu_data); // Publish IMU
#elif defined(USE_FOR_ROS2)
		imu_data.header.stamp.sec = sec;
    	imu_data.header.stamp.nanosec = nsec;
		rosIMUPublisher->publish(imu_data); // Publish IMU
#endif
	}
	virtual void OnException(const LidarInfo &lidarInfo, const Exception &e)
	{
		/*
		 *This callback function is called when the SDK sends a tip or raises an exception problem.
		 *Use another thread to respond to the exception to avoid time-consuming operations.
		 */
		if (e.GetErrorCode() > 0)
			std::cout << "[Error Code]: " << e.GetErrorCode() << " -> " << e.ToString() << std::endl;
		if (e.GetTipsCode() > 0)
			std::cout << "[Tips Code]: " << e.GetTipsCode() << " -> " << e.ToString() << std::endl;
	}
	virtual void OnDeviceInfoFrame(const LidarInfo &lidarInfo, const DeviceInfoFrame &deviceInfoFrame)
	{
/*
 *Avoid directly operating the UI in the callback function.
 */
#if defined(USE_FOR_ROS) || defined(DISABLE_PCL_ROS)
		std::string dif_value = std::string((char *)deviceInfoFrame.privateData, 1024);

		tanwaylidar_view::DIFString dif_msg;
		dif_msg.dif_packet = dif_value;
		dif_msg.header.stamp = ros::Time::now();
		dif_msg.header.frame_id = launchConfig.m_frameID;
		rosDIFPublisher.publish(dif_msg);
#endif
	}
	virtual void OnParsePcapProcess(const LidarInfo &lidarInfo, int process, uint64_t frame, uint64_t stamp) {}

#ifdef DISABLE_PCL_ROS
	void convertToPointCloud2(const UserPointCloud& pcl_cloud, 
							sensor_msgs::PointCloud2& ros_cloud, int lidarType = 21) {
		// ROS_INFO("====== Start convertToPointCloud2 ======");
		// ROS_INFO("Input point cloud size: %zu", pcl_cloud.size());
		// 1. 检查输入点云是否为空
		if (pcl_cloud.size() == 0) {
			ROS_ERROR("Input point cloud is empty!");
			return;
		}

		// 2. 设置消息头
		// ros_cloud.header.stamp = ros::Time::now();
		ros_cloud.header.stamp = ros::Time(pcl_cloud.points[0].t_sec, pcl_cloud.points[0].t_usec * 1000);    // 第一个点的时间戳
		ros_cloud.header.frame_id = launchConfig.m_frameID;
		// ROS_INFO("Header set: frame_id=%s", ros_cloud.header.frame_id.c_str());

		// 3. 设置点云基本信息
		ros_cloud.height = 1;
		ros_cloud.width = pcl_cloud.size();
		ros_cloud.is_bigendian = false;
		ros_cloud.is_dense = true;
		// ROS_INFO("Basic cloud properties set: width=%d", ros_cloud.width);

		// 4. 定义字段
		sensor_msgs::PointCloud2Modifier modifier(ros_cloud);
		// ROS_INFO("PointCloud2Modifier created");
		std::vector<sensor_msgs::PointField> fields;
		auto addField = [&fields](const std::string& name, uint8_t datatype, uint32_t offset) {
		sensor_msgs::PointField field;
		field.name = name;
		field.datatype = datatype;
		field.offset = offset;
		field.count = 1;
		fields.push_back(field);
		};
		if (lidarType == 21) {  // TW360
			// struct TW360 {
			// float x;            // 4 bytes
			// float y;            // 4 bytes
			// float z;            // 4 bytes
			// uint16_t intensity; // 2 bytes
			// uint32_t offset_time; // 4 bytes (原 UserPoint 的 offset_time)
			// uint8_t ring;       // 1 byte (对应原 channel)
			// uint8_t tag;        // 1 byte (原 tag)
			// };
			// TW360 格式字段
			addField("x", sensor_msgs::PointField::FLOAT32, 0);
			addField("y", sensor_msgs::PointField::FLOAT32, 4);
			addField("z", sensor_msgs::PointField::FLOAT32, 8);
			addField("intensity", sensor_msgs::PointField::UINT16, 12);  //填充2字节
			addField("offset_time", sensor_msgs::PointField::UINT32, 16);
			addField("ring", sensor_msgs::PointField::UINT8, 20);
			addField("tag", sensor_msgs::PointField::UINT8, 21);


			ros_cloud.fields = fields;
			ros_cloud.point_step = 22; // 应用偏移
		}
		else{
			addField("x", sensor_msgs::PointField::FLOAT32, 0);       // 0-3
			addField("y", sensor_msgs::PointField::FLOAT32, 4);       // 4-7
			addField("z", sensor_msgs::PointField::FLOAT32, 8);      // 8-12
			addField("intensity", sensor_msgs::PointField::FLOAT64, 24); // 24-31
			addField("distance", sensor_msgs::PointField::FLOAT64, 32);  // 32-39
			addField("channel", sensor_msgs::PointField::INT32, 40);    // 40-43
			addField("angle", sensor_msgs::PointField::FLOAT64, 48);    // 48-55 (修正偏移)
			addField("pulse", sensor_msgs::PointField::FLOAT64, 56);   // 56-63
			addField("echo", sensor_msgs::PointField::INT32, 64);      // 64-67
			addField("mirror", sensor_msgs::PointField::INT32, 68);     // 68-71
			addField("left_right", sensor_msgs::PointField::INT32, 72); // 72-75
			addField("block", sensor_msgs::PointField::INT32, 76);     // 76-79
			addField("t_sec", sensor_msgs::PointField::UINT32, 80);    // 80-83
			addField("t_usec", sensor_msgs::PointField::UINT32, 84);   // 84-87
			addField("offset_time", sensor_msgs::PointField::UINT32, 88); // 88-91
			addField("tag", sensor_msgs::PointField::UINT8, 92);       // 92
			addField("apd_temp", sensor_msgs::PointField::FLOAT64, 96); // 96-103 (修正偏移)
			addField("pulseCodeInterval", sensor_msgs::PointField::INT32, 104); // 104-107

			ros_cloud.fields = fields;
			ros_cloud.point_step = sizeof(UserPoint);  // 112字节
			
			
		}
		ros_cloud.row_step = ros_cloud.width * ros_cloud.point_step;

		// 5. 转换为ros消息
		modifier.resize(pcl_cloud.size());
		//ROS_INFO("Data buffer resized to %zu points", pcl_cloud.size());
		if(lidarType == 21)
		{
			// 获取指向 ros_cloud.data 的指针
			uint8_t* dst_data = ros_cloud.data.data();
			const auto first_point_time = pcl_cloud.points[0].t_sec * 1000000 + pcl_cloud.points[0].t_usec;
			for (size_t i = 0; i < pcl_cloud.size(); ++i) {
				const auto& src = pcl_cloud.points[i];
				uint8_t* dst_ptr = dst_data + i * ros_cloud.point_step;

				// 安全写入每个字段
				float x = static_cast<float>(src.x);
        		float y = static_cast<float>(src.y);
        		float z = static_cast<float>(src.z);
				memcpy(dst_ptr + 0, &x, sizeof(float));      // x
            	memcpy(dst_ptr + 4, &y, sizeof(float));      // y
            	memcpy(dst_ptr + 8, &z, sizeof(float));      // z
            
            	uint16_t intensity = src.intensity;
            	memcpy(dst_ptr + 12, &intensity, sizeof(uint16_t)); // intensity
            
            	uint32_t offset_time = (src.t_sec * 1000000 + src.t_usec - first_point_time) * 1000;  //单位ns
				
            	memcpy(dst_ptr + 16, &offset_time, sizeof(uint32_t)); // offset_time
            
            	uint8_t ring = src.channel;
            	uint8_t tag = src.confidence;
            	memcpy(dst_ptr + 20, &ring, sizeof(uint8_t));    // ring
            	memcpy(dst_ptr + 21, &tag, sizeof(uint8_t));     // tag
			}
		}
		else{
			uint8_t* dst_data = ros_cloud.data.data();
			for (size_t i = 0; i < pcl_cloud.size(); ++i) {
				const auto& src = pcl_cloud.points[i];
				uint8_t* dst_ptr = dst_data + i * ros_cloud.point_step;

				float x = static_cast<float>(src.x);
        		float y = static_cast<float>(src.y);
        		float z = static_cast<float>(src.z);
				memcpy(dst_ptr + 0, &x, sizeof(float));      // x
            	memcpy(dst_ptr + 4, &y, sizeof(float));      // y
            	memcpy(dst_ptr + 8, &z, sizeof(float));      // z


				double tmp_intensity = static_cast<double>(src.intensity);
				memcpy(dst_ptr + 24, &tmp_intensity, sizeof(double)); // intensity

				double tmp_distance = src.distance;
				memcpy(dst_ptr + 32, &tmp_distance, 8);    // distance (32-39)

				double tmp_angle = src.angle;
				memcpy(dst_ptr + 48, &tmp_angle, 8);      // angle (48-55)

				double tmp_pulse = src.pulse;
				memcpy(dst_ptr + 56, &tmp_pulse, 8);      // pulse (56-63)

				double tmp_apd_temp = src.apd_temp;
				memcpy(dst_ptr + 96, &tmp_apd_temp, 8);    // apd_temp (96-103)

				int32_t tmp_channel = src.channel;
				memcpy(dst_ptr + 40, &tmp_channel, 4); // channel

				int32_t tmp_echo = src.echo;
				memcpy(dst_ptr + 64, &tmp_echo, 4);        // echo (64-67)

				int32_t tmp_mirror = src.mirror;
				memcpy(dst_ptr + 68, &tmp_mirror, 4);      // mirror (68-71)

				int32_t tmp_left_right = src.left_right;
				memcpy(dst_ptr + 72, &tmp_left_right, 4);   // left_right (72-75)

				int32_t tmp_block = src.block;
				memcpy(dst_ptr + 76, &tmp_block, 4);       // block (76-79)

				uint32_t tmp_t_sec = src.t_sec;
				memcpy(dst_ptr + 80, &tmp_t_sec, 4);       // t_sec (80-83)

				uint32_t tmp_t_usec = src.t_usec;
				memcpy(dst_ptr + 84, &tmp_t_usec, 4);      // t_usec (84-87)

				uint32_t tmp_offset_time = src.offset_time;
				memcpy(dst_ptr + 88, &tmp_offset_time, 4); // offset_time (88-91)

				int32_t tmp_pulseCodeInterval = src.pulseCodeInterval;
				memcpy(dst_ptr + 104, &tmp_pulseCodeInterval, 4); // pulseCodeInterval (104-107)
			}		

		}
		// ROS_INFO("====== Conversion completed ======");
	}
#endif
};

int main(int argc, char **argv)
{
#if defined(USE_FOR_ROS) || defined(DISABLE_PCL_ROS)
	ros::init(argc, argv, "tanwaylidarview");
	ros::NodeHandle nh;
	// ros::Rate r(20);

	ros::NodeHandle nh_private("~");
	launchConfig.ReadLaunchParams(nh_private);
	ROS_INFO("Tanway LiDAR viewer for ROS1");
	ROS_INFO("Version 3.0.19, 2025-03-27.");
	ROS_INFO("View in rviz");

	rosPublisher = nh.advertise<sensor_msgs::PointCloud2>(launchConfig.m_topic, 1);
	rosIMUPublisher = nh.advertise<sensor_msgs::Imu>(launchConfig.m_imuTopic, 1);
	rosDIFPublisher = nh.advertise<tanwaylidar_view::DIFString>(launchConfig.m_difTopic, 1);
#elif defined(USE_FOR_ROS2)
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("tanwaylidarview2");
	// 读取Launch配置文件

	launchConfig.ReadLaunchParams(node);

	RCLCPP_INFO(node->get_logger(), "Tanway LiDAR viewer for ROS2.\n");
	RCLCPP_INFO(node->get_logger(), "Version 3.0.19, 2025-03-27.\n");
	RCLCPP_INFO(node->get_logger(), "View in rviz2.\n");

	rosPublisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(launchConfig.m_topic, 1);
	rosIMUPublisher = node->create_publisher<sensor_msgs::msg::Imu>(launchConfig.m_imuTopic, 1);
#endif

	LidarObserver lidarObserver;

	std::unique_ptr<ILidarDevice> lidar;
	// 初始化日志系统
    TWLOG& logger = TWLOG::GetInstance();
    logger.SetFileName(launchConfig.m_logFileName);  //绝对路径
	//LOG_OFF, LOG_INFO, LOG_TEST_POINT, LOG_TEST_TIME, LOG_DEBUG
	logger.SetLogLevel(logger.StringToLogLevel(launchConfig.m_logLevel));  // 设置日志等级为 LOG_TEST_POINT，低于该等级的日志都会被输出，比如LOG_INFO会被输出
    if(launchConfig.m_startLog)
		logger.Start();
	
    // 记录参数信息
    std::vector<std::string> param_names;
#if defined(USE_FOR_ROS) || defined(DISABLE_PCL_ROS)
    nh_private.getParamNames(param_names);
#elif defined(USE_FOR_ROS2)
    auto parameter_descriptors = node->list_parameters({}, 0); // 获取所有参数的描述符
    for (const auto& descriptor : parameter_descriptors.names) {
        param_names.push_back(descriptor);
    }
#endif

    for (const auto& param_name : param_names) {
        std::string param_value;
#if defined(USE_FOR_ROS) || defined(DISABLE_PCL_ROS)
        if (nh_private.hasParam(param_name)) {
            XmlRpc::XmlRpcValue value;
            nh_private.getParam(param_name, value);
            if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
                param_value = std::string(value);  // 直接赋值
            } else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                param_value = std::to_string(static_cast<int>(value));
            } else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                param_value = std::to_string(static_cast<double>(value));
            } else if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
                param_value = value ? "true" : "false";
            } else {
                param_value = "<unknown type>";
            }
        } else {
            param_value = "<not set>";
        }
#elif defined(USE_FOR_ROS2)
        if (node->has_parameter(param_name)) {
            auto param_type = node->get_parameter(param_name).get_type();
            switch (param_type) {
                case rclcpp::ParameterType::PARAMETER_STRING:
                    param_value = node->get_parameter(param_name).as_string();
                    break;
                case rclcpp::ParameterType::PARAMETER_INTEGER:
                    param_value = std::to_string(node->get_parameter(param_name).as_int());
                    break;
                case rclcpp::ParameterType::PARAMETER_DOUBLE:
                    param_value = std::to_string(node->get_parameter(param_name).as_double());
                    break;
                case rclcpp::ParameterType::PARAMETER_BOOL:
                    param_value = node->get_parameter(param_name).as_bool() ? "true" : "false";
                    break;
                default:
                    param_value = "<unknown type>";
                    break;
            }
        } else {
            param_value = "<not set>";
        }
#endif
        logger.LogInfo(__FILE__, __LINE__, __PRETTY_FUNCTION__, "Parameter: %s = %s", param_name.c_str(), param_value.c_str());
    }

	// on-line
	if ("on-line" == launchConfig.m_connectType)
	{
		lidar = ILidarDevice::Create(launchConfig.m_lidarHost, launchConfig.m_localHost, launchConfig.m_localPointCloudPort, launchConfig.m_localDIFPort, &lidarObserver, (LidarType)(launchConfig.m_lidarType),false,0,launchConfig.m_localIMUPort);
	}
	// off-line
	else
	{
		lidar = ILidarDevice::Create(launchConfig.m_filePath, launchConfig.m_lidarHost, launchConfig.m_localPointCloudPort, launchConfig.m_localDIFPort, &lidarObserver, (LidarType)(launchConfig.m_lidarType), true,0,launchConfig.m_localIMUPort);
		
	}

	auto algo = ILidarAlgo::Create((LidarType)launchConfig.m_lidarType, launchConfig.m_AlgoTable);

	lidar->SetLidarAlgo(algo.get());


	lidar->SetCoreHorAngleOffset(launchConfig.m_coreHorAngleOffsetL, launchConfig.m_coreHorAngleOffsetR);
	lidar->SetCoreVerAngleOffset(launchConfig.m_coreVerAngleOffsetL, launchConfig.m_coreVerAngleOffsetR);

	lidar->SetMirrorHorAngleOffset(launchConfig.m_mirrorHorAngleOffsetA, launchConfig.m_mirrorHorAngleOffsetB, launchConfig.m_mirrorHorAngleOffsetC);
	lidar->SetMirrorVerAngleOffset(launchConfig.m_mirrorVerAngleOffsetA, launchConfig.m_mirrorVerAngleOffsetB, launchConfig.m_mirrorVerAngleOffsetC);

	// 新加：把时间戳类型写到成员变量
	lidar->SetTimeStampType(launchConfig.m_timeStamp);
	lidar->SetLidarTime(launchConfig.m_useLidarTime);

	lidar->SetFrameSplit(launchConfig.m_cycleCountFrameSplit);
	lidar->SetTimeWindowMode(launchConfig.m_useTimeWindow);
	lidar->Start();

#if defined(USE_FOR_ROS) || defined(DISABLE_PCL_ROS)
	while (ros::ok())
#elif defined(USE_FOR_ROS2)
	while (rclcpp::ok())
#endif
	{
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	return 0;
}
