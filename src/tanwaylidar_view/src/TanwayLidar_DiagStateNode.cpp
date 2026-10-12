/************************************************
 *  Copyright (C) 2023 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-09-2023
 *  Edited on: 16-09-2022
 *  Author: LHB

 *  Node for Tanway 3D LIDARs Diagnostic State
**************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include "LaunchConfig.h"
#include "tanwaylidar_view/DiagState.h"
#include "tanwaylidar_view/DIFString.h"

ros::Publisher diagPublisher;
std::string difTopic;
std::string diagTopic;

unsigned int FourHexToInt(unsigned char high, unsigned char highmiddle, unsigned char middle, unsigned char low)
{
	unsigned int addr = low & 0xFF;
	addr |= ((middle << 8) & 0xFF00);
	addr |= ((highmiddle << 16) & 0xFF0000);
	addr |= ((high << 24) & 0xFF000000);
	return addr;
}

unsigned int TwoHextoInt(unsigned char high, unsigned char low)
{
	unsigned int addr = low & 0xFF;
	addr |= ((high << 8) & 0XFF00);
	return addr;
}


void difCallback(const tanwaylidar_view::DIFString::ConstPtr& msg)
{
	const char* dif = msg->dif_packet.c_str();
	
	int lidarTpye = (unsigned char)(dif[11]);
	int lidarBatchNumber = (unsigned char)(dif[10]);
	uint32_t lidarNumber = FourHexToInt(dif[12], dif[13], dif[14], dif[15]);
	uint32_t psVersion = (dif[17] << 16) | (dif[18] << 8) | dif[19];
	uint32_t plVersion = (dif[21] << 16) | (dif[22] << 8) | dif[23];
	uint32_t kernalVersion = (dif[25] << 16) | (dif[26] << 8) | dif[27];
	uint32_t protocolVersion = (dif[28] << 8) | dif[29];
	uint32_t workMode = dif[32];
	uint32_t workStatus = dif[33];
	uint32_t timeSync = dif[37];

	uint64_t errorCode = 0;
	unsigned long long classifier_LIDAR_ERROR = 0x7E0F0F00020F30F6;

	struct hwcode {
		unsigned int low = 0x00;
		unsigned int high = 0x00;
	};
	struct hwcode s;
	unsigned long long *code = (unsigned long long *)&s;

	// byte[0..7]: 40, 60, 00, 00, 00, 01, 00, 2a.
	s.low |= (dif[391] & 0x000000ff);
	s.low |= ((dif[390] << 8) & 0x0000ff00);
	s.low |= ((dif[389] << 16) & 0x00ff0000);
	s.low |= ((dif[388] << 24) & 0xff000000);
	s.high |= (dif[387] & 0x000000ff);
	s.high |= ((dif[386] << 8) & 0x0000ff00);
	s.high |= ((dif[385] << 16) & 0x00ff0000);
	s.high |= ((dif[384] << 24) & 0xff000000);

	errorCode |= s.high;
	errorCode = errorCode << 32;
	errorCode |= s.low;
	
	//errorCode &= classifier_LIDAR_ERROR;

	uint32_t occludedNumber = TwoHextoInt(dif[508 + 34], dif[508 + 35]);

	tanwaylidar_view::DiagState diag;
	diag.header.stamp = msg->header.stamp;
	diag.header.frame_id = msg->header.frame_id;
	diag.device_code = lidarNumber;
	diag.ps_version = psVersion;
	diag.pl_version = plVersion;
	diag.kernal_version = kernalVersion;
	diag.protocol_version = protocolVersion;
	diag.work_mode = workMode;
	diag.running_state = workStatus;
	diag.time_sync_state = (s.low & 0x20) >> 5;
	diag.error_code = errorCode;
	diag.reserved1 = lidarBatchNumber;
	diag.reserved2 = occludedNumber;

    diagPublisher.publish(diag);
}

int main(int argc, char** argv) 
{
	ros::Subscriber rosSubscriber;
    ros::init(argc, argv,"tanwaylidarview_dif");
    ros::NodeHandle nd("~");

	nd.param<std::string>("topic_diag", diagTopic, "/tanwaylidar_diagstate");
	nd.param<std::string>("topic_dif", difTopic, "/tanwaylidar_dif");

    rosSubscriber = nd.subscribe<tanwaylidar_view::DIFString>(difTopic, 1, &difCallback);
	diagPublisher = nd.advertise<tanwaylidar_view::DiagState>(diagTopic, 1);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;

}
