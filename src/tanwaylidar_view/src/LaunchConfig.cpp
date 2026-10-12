/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 30-01-2021
 *  Author: LN

 *  config setting for Tanway LIDARs
**************************************************/
#include <LaunchConfig.h>

#ifdef USE_FOR_ROS2
void ReadSingleParamsString(const std::shared_ptr<rclcpp::Node> &node, const std::string &name, std::string &parameter, std::string default_value)
{
	node->declare_parameter<std::string>(name, default_value);
	if (node->get_parameter(name, parameter))
		RCLCPP_INFO(node->get_logger(), "Parameter '%s' value: %s", name.c_str(), parameter.c_str());
}

void ReadSingleParamsInt(const std::shared_ptr<rclcpp::Node> &node, const std::string &name, int &parameter, int default_value)
{
	node->declare_parameter<int>(name, default_value);
	if (node->get_parameter(name, parameter))
		RCLCPP_INFO(node->get_logger(), "Parameter '%s' value: %d", name.c_str(), parameter);
}

void ReadSingleParamsDouble(const std::shared_ptr<rclcpp::Node> &node, const std::string &name, double &parameter, double default_value)
{
	node->declare_parameter<double>(name, default_value);
	if (node->get_parameter(name, parameter))
		RCLCPP_INFO(node->get_logger(), "Parameter '%s' value: %.4f", name.c_str(), parameter);
}
void ReadSingleParamsBool(const std::shared_ptr<rclcpp::Node> &node, const std::string &name, bool &parameter, bool default_value)
{

    node->declare_parameter<bool>(name, default_value);
    if (node->get_parameter(name, parameter))
        RCLCPP_INFO(node->get_logger(), "Parameter '%s' value: %s", name.c_str(), parameter ? "true" : "false");
}
#endif

LaunchConfig::LaunchConfig()
{
}

LaunchConfig::~LaunchConfig()
{
}

#if defined(USE_FOR_ROS) || defined(DISABLE_PCL_ROS)
void LaunchConfig::ReadLaunchParams(ros::NodeHandle &nh_private)
{
	nh_private.param<std::string>("ConnectType", m_connectType, "on-line"); //"on-line"是当参数不存在于参数服务器时使用的默认值
	nh_private.param<std::string>("PcapFilePath", m_filePath, "");
	nh_private.param<std::string>("AlgoTablePath", m_AlgoTable, "");
	nh_private.param<std::string>("LocalHost", m_localHost, "192.168.111.204");
	nh_private.param<int>("LocalPointloudPort", m_localPointCloudPort, 5600);
	nh_private.param<int>("LocalDIFPort", m_localDIFPort, 5700);
	nh_private.param<int>("LocalIMUPort", m_localIMUPort, 5700);

	nh_private.param<std::string>("LidarHost", m_lidarHost, "192.168.111.51");
	// nh_private.param<int>("LidarPort", m_lidarPort, 5050);

	nh_private.param<std::string>("frame_id", m_frameID, "TanwayTP");
	nh_private.param<std::string>("topic", m_topic, "/tanwaylidar_pointcloud");
	nh_private.param<std::string>("imu_topic", m_imuTopic, "/tanwaylidar_imu");

	nh_private.param<int>("LidarType", m_lidarType, -1);

	

	// Scope-192
	if (LT_Scope192 == m_lidarType)
	{
		nh_private.param<double>("CorrectedAngle1", m_correctedAngle1, 0);
		nh_private.param<double>("CorrectedAngle2", m_correctedAngle2, -0.12);
		nh_private.param<double>("CorrectedAngle3", m_correctedAngle3, -0.24);
	}
	// Focus
	else if (LT_FocusB1 == m_lidarType)
	{
		nh_private.param<bool>("bJointabc", m_bJointabc, false);
		nh_private.param<double>("jointabc_node1", m_jointabc_node1, 1.0);
		nh_private.param<double>("jointabc_node2", m_jointabc_node2, 10.0);
		nh_private.param<int>("jointabc_one_face", m_jointabc_one_face, 1);
		nh_private.param<int>("jointabc_two_face", m_jointabc_two_face, 1);
	}

	nh_private.param<double>("CoreHorAngleOffsetL", m_coreHorAngleOffsetL, 0);
	nh_private.param<double>("CoreHorAngleOffsetR", m_coreHorAngleOffsetR, 0);
	nh_private.param<double>("CoreVerAngleOffsetL", m_coreVerAngleOffsetL, 0);
	nh_private.param<double>("CoreVerAngleOffsetR", m_coreVerAngleOffsetR, 0);

	nh_private.param<double>("MirrorVerAngleOffsetA", m_mirrorVerAngleOffsetA, 0);
	nh_private.param<double>("MirrorVerAngleOffsetB", m_mirrorVerAngleOffsetB, 0);
	nh_private.param<double>("MirrorVerAngleOffsetC", m_mirrorVerAngleOffsetC, 0);

	nh_private.param<double>("MirrorHorAngleOffsetA", m_mirrorHorAngleOffsetA, 0);
	nh_private.param<double>("MirrorHorAngleOffsetB", m_mirrorHorAngleOffsetB, 0);
	nh_private.param<double>("MirrorHorAngleOffsetC", m_mirrorHorAngleOffsetC, 0);

	// 帧时间戳
	nh_private.param<std::string>("FrameTimeStampType", m_timeStamp, "last_point");
	nh_private.param<bool>("UseLidarClock", m_useLidarTime, true);
	nh_private.param<bool>("CycleCountFrameSplit", m_cycleCountFrameSplit, false);
	//时间分帧模式
	nh_private.param<bool>("UseTimeWindow", m_useTimeWindow, false);
	
	nh_private.param<std::string>("LogFileName", m_logFileName, "log_time.log");
	nh_private.param<std::string>("LogLevel", m_logLevel, "LOG_OFF");
	nh_private.param<bool>("StartLog", m_startLog, false);

}
#elif defined(USE_FOR_ROS2)
void LaunchConfig::ReadLaunchParams(const std::shared_ptr<rclcpp::Node> &node)
{
	ReadSingleParamsString(node, "ConnectType", m_connectType, "on-line");
	ReadSingleParamsString(node, "PcapFilePath", m_filePath, "");
	ReadSingleParamsString(node, "AlgoTablePath", m_AlgoTable, "");
	ReadSingleParamsString(node, "LocalHost", m_localHost, m_localHost);
	ReadSingleParamsInt(node, "LocalPointloudPort", m_localPointCloudPort, 5600);
	ReadSingleParamsInt(node, "LocalDIFPort", m_localDIFPort, 5700);
	ReadSingleParamsInt(node, "LocalIMUPort", m_localIMUPort, 5700);
	ReadSingleParamsString(node, "LidarHost", m_lidarHost, m_lidarHost);

	ReadSingleParamsString(node, "frame_id", m_frameID, "TanwayTP");
	ReadSingleParamsString(node, "topic", m_topic, "/tanwaylidar_pointcloud");
	ReadSingleParamsString(node, "imu_topic", m_imuTopic, "/tanwaylidar_imu");
	ReadSingleParamsInt(node, "LidarType", m_lidarType, -1);

	

	ReadSingleParamsDouble(node, "CoreHorAngleOffsetL", m_coreHorAngleOffsetL, 0);
	ReadSingleParamsDouble(node, "CoreHorAngleOffsetR", m_coreHorAngleOffsetR, 0);
	ReadSingleParamsDouble(node, "CoreVerAngleOffsetL", m_coreVerAngleOffsetL, 0);
	ReadSingleParamsDouble(node, "CoreVerAngleOffsetR", m_coreVerAngleOffsetR, 0);

	ReadSingleParamsDouble(node, "MirrorVerAngleOffsetA", m_mirrorVerAngleOffsetA, 0);
	ReadSingleParamsDouble(node, "MirrorVerAngleOffsetB", m_mirrorVerAngleOffsetB, 0);
	ReadSingleParamsDouble(node, "MirrorVerAngleOffsetC", m_mirrorVerAngleOffsetC, 0);

	ReadSingleParamsDouble(node, "MirrorHorAngleOffsetA", m_mirrorHorAngleOffsetA, 0);
	ReadSingleParamsDouble(node, "MirrorHorAngleOffsetB", m_mirrorHorAngleOffsetB, 0);
	ReadSingleParamsDouble(node, "MirrorHorAngleOffsetC", m_mirrorHorAngleOffsetC, 0);

	// 帧时间戳
	ReadSingleParamsString(node,"FrameTimeStampType", m_timeStamp, "last_point");
	ReadSingleParamsBool(node,"UseLidarClock", m_useLidarTime, true);
	ReadSingleParamsBool(node,"CycleCountFrameSplit", m_cycleCountFrameSplit, false);
	//时间分帧模式
	ReadSingleParamsBool(node,"UseTimeWindow", m_useTimeWindow, false);


	ReadSingleParamsString(node, "LogFileName", m_logFileName, "log_time.log");
	ReadSingleParamsString(node, "LogLevel", m_logLevel, "LOG_OFF");
	ReadSingleParamsBool(node, "StartLog", m_startLog, false);

}
#endif
