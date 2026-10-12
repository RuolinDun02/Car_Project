#ifndef TANWAY_LINE_EXTRACTOR__TANWAY_LINE_EXTRACTOR_HPP_
#define TANWAY_LINE_EXTRACTOR__TANWAY_LINE_EXTRACTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "tanway_line_extractor/msg/line_segment3_d.hpp"
#include "tanway_line_extractor/msg/line_segment3_d_array.hpp"

#include "laser_line_extraction/line_extraction.h"
#include "laser_line_extraction/line.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace tanway_line_extractor
{

/** @brief 单层内一个点。
 *  优化：所有字段改用 float，结构体从 48 字节降到 24 字节。
 *       点云原始精度就是 float32，用 double 存没有收益。
 */
struct Point2D
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float range{0.0f};      // sqrt(x^2 + y^2)
  float bearing{0.0f};    // atan2(y, x)
  float intensity{0.0f};
};

class TanwayLineExtractor : public rclcpp::Node
{
public:
  TanwayLineExtractor();

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void extractFromRing(
    uint16_t ring,
    std::vector<Point2D> & pts,
    const rclcpp::Time & stamp,
    msg::LineSegment3DArray & out,
    visualization_msgs::msg::MarkerArray & markers,
    int & marker_id);

  void configureExtractor(line_extraction::LineExtraction & ex) const;

  /* ===== 参数：话题与坐标系 ===== */
  std::string cloud_topic_;
  std::string lines_topic_;
  std::string markers_topic_;
  std::string frame_id_;

  /* ===== 参数：点云过滤 ===== */
  double min_range_{0.3};
  double max_range_{30.0};
  int    min_intensity_{0};
  int    max_intensity_{1000};
  int    ring_min_{1};
  int    ring_max_{48};

  /* ===== 参数：算法 ===== */
  double bearing_std_dev_{0.001};
  double range_std_dev_{0.02};
  double ls_angle_thresh_{0.0001};
  double ls_radius_thresh_{0.0001};
  double max_line_gap_{0.05};
  double min_line_length_{0.25};
  int    min_line_points_{8};
  double min_split_dist_{0.04};
  double outlier_dist_{0.06};

  /* ===== 参数：性能优化（新增）===== */
  // 处理频率上限 Hz。雷达 10Hz，设 5.0 则每两帧处理一帧。
  // 设 0 表示不限制（每帧都处理）。
  double processing_rate_{10.0};
  // 每层点数上限。超过则原地等间隔下采样。
  // 设 0 表示不限制。
  int    max_points_per_ring_{1800};
  // 点云订阅队列深度，越小越省内存。
  int    cloud_queue_size_{3};

  /* ===== 复用缓冲区（避免每帧/每层重复分配）===== */
  // 用固定大小数组代替 std::map，O(1) 索引且无需红黑树
  static constexpr std::size_t MAX_RING = 256;
  std::array<std::vector<Point2D>, MAX_RING> ring_pts_;

  // LineExtraction 的输入缓冲，每层 clear() 后复用
  std::vector<double>       bearings_;
  std::vector<double>       cos_bearings_;
  std::vector<double>       sin_bearings_;
  std::vector<double>       ranges_;
  std::vector<unsigned int> indices_;

  // 参数配置只做一次
  bool extractor_configured_{false};

  // 降频控制时间戳
  rclcpp::Time last_process_time_;

  /* ===== ROS 接口 ===== */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<msg::LineSegment3DArray>::SharedPtr lines_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  line_extraction::LineExtraction extractor_;
};

}  // namespace tanway_line_extractor

#endif