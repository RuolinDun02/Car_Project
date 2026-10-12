/*
* Software License Agreement (BSD License)
*   
*  Copyright (c) Tanway science and technology co., LTD.
*
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without modification, 
*  are permitted provided  that the following conditions are met:
*
*   1.Redistributions of source code must retain the above copyright notice, 
*     this list of conditions and the following disclaimer.
*
*   2.Redistributions in binary form must reproduce the above copyright notice, 
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*     
*   3.Neither the name of the copyright holder(s) nor the names of its  contributors
*     may be used to endorse or promote products derived from this software without 
*     specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
*  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "../include/ILidarDevice.h"
#include "../include/ILidarAlgo.h"
#include "iostream"
#include <thread>
#include <iomanip>
#include <fstream>
#include <string>
#include <map>
#include "../include/json.hpp"


using namespace tanway;
using json = nlohmann::json;

// 雷达类型映射表
std::map<std::string, LidarType> lidarTypeMap = {
    {"LT_Tensor16", LT_Tensor16},
    {"LT_Tensor32", LT_Tensor32},
    {"LT_Scope192", LT_Scope192},
    {"LT_Duetto", LT_Duetto},
    {"LT_TempoA1", LT_TempoA1},
    {"LT_TempoA2", LT_TempoA2},
    {"LT_ScopeMini", LT_ScopeMini},
    {"LT_TempoA3", LT_TempoA3},
    {"LT_TempoA4", LT_TempoA4},
    {"LT_Tensor48", LT_Tensor48},
    {"LT_Tensor48_Depth", LT_Tensor48_Depth},
    {"LT_Scope256", LT_Scope256},
    {"LT_Scope256_Depth", LT_Scope256_Depth},
    {"LT_FocusB1", LT_FocusB1},
    {"LT_Scope256_SmallBlind", LT_Scope256_SmallBlind},
    {"LT_FocusB2_B3_MP", LT_FocusB2_B3_MP},
    {"LT_Scope128H", LT_Scope128H},
    {"LT_Scope128", LT_Scope128},
    {"LT_Scope128F", LT_Scope128F},
    {"LT_FocusB2_64", LT_FocusB2_64},
    {"LT_FocusT", LT_FocusT},
    {"LT_TW360", LT_TW360}
};

// Load configuration file
json loadConfig(const std::string& configPath) {
	std::ifstream configFile(configPath);
	if (!configFile.is_open()) {
		std::cerr << "Unable to open configuration file: " << configPath << std::endl;
		return json{};
	}
	
	try {
		json config;
		configFile >> config;
		return config;
	} catch (const std::exception& e) {
		std::cerr << "Error parsing configuration file: " << e.what() << std::endl;
		return json{};
	}
}

class LidarObserver : public ILidarObserver{
public:
  virtual void OnPointCloud(const LidarInfo &lidarInfo, const UserPointCloud &pointCloud){
	/*
	*The point cloud struct uses a const reference. 
	*Please copy the point cloud data to another thread for use.
	*Avoid directly operating the UI in the callback function.
	*/
	std::cout << "width:" << pointCloud.width 
			  << " height:" << pointCloud.height 
			  << " point cloud size: " << pointCloud.points.size() << std::endl;
  }
  virtual void OnIMU(const LidarInfo &lidarInfo, const IMUData &imu){
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

  }
  virtual void OnException(const LidarInfo &lidarInfo, const Exception &e){
	/* 
	*This callback function is called when the SDK sends a tip or raises an exception problem.
	*Use another thread to respond to the exception to avoid time-consuming operations.
	*/
	if (e.GetErrorCode() > 0)
		std::cout << "[Error Code]: " << e.GetErrorCode() << " -> " << e.ToString() << std::endl;
	if (e.GetTipsCode() > 0)
		std::cout << "[Tips Code]: " << e.GetTipsCode() << " -> " << e.ToString() << std::endl;	

  }
  virtual void OnDeviceInfoFrame(const LidarInfo &lidarInfo, const DeviceInfoFrame &deviceInfoFrame){
	  //std::cout << "  lidar_id:" << lidarInfo.lidarID << std::endl;
  }
  virtual void OnParsePcapProcess(const LidarInfo &lidarInfo, int process, uint64_t frame, uint64_t stamp){}
};

int main(int argc, char* argv[])
{
	std::string configPath = "../config/lidar_config.json";  // Using default json file

	if (argc > 1) {
        configPath = argv[1];
    }

	// Load configuration file
	json config = loadConfig(configPath);
	if (config.empty()) {
		std::cerr << "Configuration loading failed, exiting program" << std::endl;
		return 1;
	}
	
	// Display loaded configuration
	std::cout << "Configuration file loaded: " << configPath << std::endl;

	LidarObserver lidarObserver;
	std::shared_ptr<ILidarDevice> lidar = nullptr;


	LidarType lidarType = LT_Duetto; 
    if (config.contains("lidar_type")) {
        std::string typeStr = config["lidar_type"];
        if (lidarTypeMap.find(typeStr) != lidarTypeMap.end()) {
            lidarType = lidarTypeMap[typeStr];
        } else {
			std::cout << "Warning: Unknown lidar type \"" << typeStr << "\", using default type Duetto" << std::endl;
        }
    }

	std::string mode = config.value("lidar_mode", "on-line");

	if (mode == "on-line") {
		auto& online = config["on-line_config"];
        std::string lidarIp = online.value("lidar_ip", "192.168.111.51");
        std::string hostIp = online.value("host_ip", "192.168.111.204");
        int dataPort = online.value("data_port", 5600);
        int difPort = online.value("dif_port", 5700);
		int imuPort = online.value("imu_port", 5700);
		int lidarID = online.value("lidar_id", 0);
		lidar = ILidarDevice::Create(lidarIp.c_str(), hostIp.c_str(), dataPort, difPort,
                                         &lidarObserver, lidarType, false, lidarID, imuPort);
		
	}
	else if (mode == "off-line") {
        auto& offline = config["off-line_config"];
        std::string pcapPath = offline.value("pcap_file_path", "test.pcap");
        std::string lidarIp = offline.value("lidar_ip", "192.168.111.51");
        int dataPort = offline.value("data_port", 5600);
        int difPort = offline.value("dif_port", 5700);
		int imuPort = offline.value("imu_port", 5700);
		int lidarID = offline.value("lidar_id", 0);
		bool repeat = offline.value("need_replay", true);
        
        lidar = ILidarDevice::Create(pcapPath.c_str(), lidarIp.c_str(), dataPort, difPort,
                                     &lidarObserver, lidarType, true,lidarID,imuPort);
       
    } else {
		std::cerr << "Unknown lidar mode: " << mode << std::endl;
        return 1;
    }
	std::unique_ptr<ILidarAlgo> algo = nullptr;;
	if (config.contains("algo_config") && config["algo_config"].value("enabled", true)) {
        std::string algoPath = config["algo_config"].value("algo_table_path", "../config/algo_table.json");
		algo = ILidarAlgo::Create(lidarType, algoPath.c_str());
        lidar->SetLidarAlgo(algo.get());
		std::cout << "Algorithm configuration loaded: " << algoPath << std::endl;
    }
	
	
	
	//start lidar
	lidar->Start();


	//quit
	// 以下代码演示了每三秒进行一次雷达的开启和关闭操作。
    // 正常情况下，用户无需频繁执行此操作。调用 lidar->Start() 启动雷达后，根据需要调用 lidar->Stop() 停止雷达即可。
	bool run_t = false;
	while (true)
	{
		std::this_thread::sleep_for(std::chrono::seconds(3));
		if (run_t)
		{
			lidar->Start();
			std::cout << "===========: start()" << std::endl;
			run_t = false;
		}
		else
		{
			lidar->Stop();
			std::cout << "===========: stop()" << std::endl;
			run_t = true;
		}
	}

    return 0;
}

