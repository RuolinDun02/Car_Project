#pragma once
#include "common.h"
using namespace tanway;

#ifdef USE_FOR_LIDAR_VIEW

#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>

// PCL扩展点云数据
struct UserPoint // 定义点类型结构
{
  PCL_ADD_POINT4D; // 该点类型有4个元素
  // PCL_ADD_RGB;
  float distance = 0; // 距离
  int channel = 0;    // 通道
  float angle = 0;    // 水平角度，udp数据协议中的水平角度

  float pulse = 0;              // 脉宽
  float intensity = 0;          // 强度
  int echo = 1;                 // 回波次数
  int mirror = 0;               // 镜面值
  int left_right = 0;           // 左右机芯 0：右；1：左
  int confidence = 0;           // 置信度
  float speed = 0;              // 速度预估值
  double apd_temp = 0;          // APD 温度
  unsigned short up_edge = 0;   // 上升沿 数量
  unsigned short down_edge = 0; // 下降沿 数量
  unsigned short edge1 = 0;     // 上升沿1
  unsigned short edge2 = 0;     // 下降沿1
  unsigned short edge3 = 0;     // 上升沿2
  unsigned short edge4 = 0;     // 下降沿2
  unsigned short edge5 = 0;     // 上升沿3
  unsigned short edge6 = 0;     // 下降沿3
  int ch1 = 0;
  bool ch1_value = false;
  int ch2 = 0;
  bool ch2_value = false;
  int t_sec = 0;
  int t_usec = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 确保new操作符对齐操作
} EIGEN_ALIGN16;                  // 强制SSE对齐

typedef class pcl::PointCloud<UserPoint> UserPointCloud;
#define make_shared_point_cloud std::make_shared<UserPointCloud>()
POINT_CLOUD_REGISTER_POINT_STRUCT(
    UserPoint, // 注册点类型宏
    (float, x, x)(float, y, y)(float, z, z)(float, distance,
                                            distance)(int, channel, channel)(
        float, angle, angle)(float, pulse, pulse)(float, intensity, intensity)(
        int, echo, echo)(int, mirror, mirror)(int, left_right, left_right)(
        int, confidence, confidence)(float, speed, speed)(unsigned short,
                                                          up_edge, up_edge)(
        unsigned short, down_edge,
        down_edge)(unsigned short, edge1, edge1)(unsigned short, edge2, edge2)(
        unsigned short, edge3, edge3)(unsigned short, edge4,
                                      edge4)(unsigned short, edge5, edge5))

#elif USE_FOR_ROS

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes

struct UserPoint {
  PCL_ADD_POINT4D;

  float intensity;
  int channel;
  float angle;
  int echo;
  int block;           /*For duetto*/
  unsigned int t_sec;  /* The value represents seconds since 1900-01-01 00:00:00
                          (the UNIX epoch).*/
  unsigned int t_usec; /* remaining microseconds */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    UserPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        int, channel, channel)(float, angle, angle)(int, echo, echo)(
        int, block, block)(unsigned int, t_sec, t_sec)(unsigned int, t_usec,
                                                       t_usec))

typedef class pcl::PointCloud<UserPoint> UserPointCloud;
#define make_shared_point_cloud boost::make_shared<UserPointCloud>()

#elif USE_FOR_ROS2

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

struct UserPoint {
  PCL_ADD_POINT4D;

  float intensity;
  int channel;
  float angle;
  int echo;
  int block;           /*For duetto*/
  unsigned int t_sec;  /* The value represents seconds since 1900-01-01 00:00:00
                          (the UNIX epoch).*/
  unsigned int t_usec; /* remaining microseconds */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    UserPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        int, channel, channel)(float, angle, angle)(int, echo, echo)(
        int, block, block)(unsigned int, t_sec, t_sec)(unsigned int, t_usec,
                                                       t_usec))

typedef class pcl::PointCloud<UserPoint> UserPointCloud;

#ifdef PCL_USE_BOOST
#define make_shared_point_cloud boost::make_shared<UserPointCloud>()
#else
#define make_shared_point_cloud std::make_shared<UserPointCloud>()
#endif

#else
struct UserPoint // 定义点类型结构
{
  double x;
  double y;
  double z;

  double intensity = 0;
  double distance = 0;
  int channel = 0;
  double angle = 0;
  double pulse = 0;

  int echo = 1;
  int mirror = 0;
  int left_right = 0;
  int block = 0;
  unsigned int t_sec = 0;
  unsigned int t_usec = 0;
  uint32_t offset_time = 0; // offset time in nanoseconds
  
  uint8_t confidence = 0; // confidence for point cloud
  double apd_temp = 0;

  int pulseCodeInterval = 0;
};

typedef class PointCloud<UserPoint> UserPointCloud;
#define make_shared_point_cloud std::make_shared<UserPointCloud>()

#endif
