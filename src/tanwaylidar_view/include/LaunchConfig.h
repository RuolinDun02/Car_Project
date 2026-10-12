
/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-01-2021
 *  Edited on: 15-03-2022
 *  Author: LN

 *  config setting for Tanway LIDARs
**************************************************/

#ifndef LAUNCHCONFIG_H_
#define LAUNCHCONFIG_H_

#include <strings.h>
#if defined(USE_FOR_ROS) || defined(DISABLE_PCL_ROS)
#include <ros/ros.h>
#elif defined(USE_FOR_ROS2)
#include "rclcpp/rclcpp.hpp"
#endif

#include "../sdk/include/ILidarDevice.h"
#include "../sdk/include/ILidarAlgo.h"

class LaunchConfig
{
public:
	LaunchConfig();
	~LaunchConfig();

#if defined(USE_FOR_ROS) || defined(DISABLE_PCL_ROS)
	void ReadLaunchParams(ros::NodeHandle &nh_private);
#elif defined(USE_FOR_ROS2)
	void ReadLaunchParams(const std::shared_ptr<rclcpp::Node> &nh_private);
#endif

public:
	std::string m_connectType = "";
	std::string m_filePath = "";
	std::string m_AlgoTable = "";
	std::string m_localHost = "";
	std::string m_lidarHost = "";
	int m_localPointCloudPort = -1;
	int m_localDIFPort = -1;
	int m_localIMUPort = -1;
	// int m_lidarPort = -1;
	std::string m_frameID = "TanwayTP";
	std::string m_topic = "/tanwaylidar_pointcloud";
	std::string m_imuTopic = "/tanwaylidar_imu";
	std::string m_difTopic = "/tanwaylidar_dif";

	int m_lidarType = -1;

	double m_correctedAngle1 = 0;
	double m_correctedAngle2 = 0;
	double m_correctedAngle3 = 0;


	// jointabc
	bool m_bJointabc = false;
	double m_jointabc_node1 = 0;
	double m_jointabc_node2 = 0;
	int m_jointabc_one_face = 0;
	int m_jointabc_two_face = 0;

	double m_coreHorAngleOffsetL = 0.0;
	double m_coreHorAngleOffsetR = 0.0;
	double m_coreVerAngleOffsetL = 0.0;
	double m_coreVerAngleOffsetR = 0.0;
	double m_mirrorVerAngleOffsetA = 0.0; // 镜面A面垂直角度偏移
	double m_mirrorVerAngleOffsetB = 0.0;
	double m_mirrorVerAngleOffsetC = 0.0;
	double m_mirrorHorAngleOffsetA = 0.0; // 镜面A面水平角度偏移
	double m_mirrorHorAngleOffsetB = 0.0;
	double m_mirrorHorAngleOffsetC = 0.0;

	// 发布帧时间戳的类型
	std::string m_timeStamp = "";
	// 点时间戳的类型
	bool m_useLidarTime = true;
	// 是否使用新的分帧逻辑（周期计数）
	bool m_cycleCountFrameSplit = false;
	// 是否使用时间分帧模式
	bool m_useTimeWindow = false;

	//日志系统相关配置
	std::string m_logFileName = "";
	std::string m_logLevel = "";
	bool m_startLog = false;


};

#endif
