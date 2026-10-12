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

#include <Eigen/Dense>

namespace tanway_line_extractor
{

/** @brief 单点（3D + 强度） */
struct Point3D
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{0.0f};
};

class TanwayLineExtractor : public rclcpp::Node
{
public:
  TanwayLineExtractor();

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void extractPerRing(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    msg::LineSegment3DArray & out,
    visualization_msgs::msg::MarkerArray & markers);

  void extractFromRing(
    uint16_t ring,
    std::vector<Point3D> & pts,
    const rclcpp::Time & stamp,
    msg::LineSegment3DArray & out,
    visualization_msgs::msg::MarkerArray & markers,
    int & marker_id);

  void extract3DCluster(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    msg::LineSegment3DArray & out,
    visualization_msgs::msg::MarkerArray & markers);

  void configureExtractor(line_extraction::LineExtraction & ex) const;

  /** @brief 把高反光点发布为 PointCloud2（调试用） */
  void publishReflectiveCloud(
    const std::vector<Point3D> & pts,
    const rclcpp::Time & stamp,
    const std::string & frame_id);

  /* ===== 话题与坐标系 ===== */
  std::string cloud_topic_;
  std::string lines_topic_;
  std::string markers_topic_;
  std::string frame_id_;

  /* ===== 提取模式 ===== */
  std::string extraction_mode_;

  /* ===== 点云过滤 ===== */
  double min_range_{0.3};
  double max_range_{30.0};
  int    min_intensity_{0};
  int    max_intensity_{1000};
  int    ring_min_{1};
  int    ring_max_{48};

  /* ===== 3D 聚类模式参数 ===== */
  double reflective_threshold_{180.0};
  double cluster_eps_{0.08};
  int    cluster_min_pts_{15};
  double linearity_threshold_{8.0};

  /* ===== 反光条长度限制 ===== */
  double refl_line_min_length_{0.20};
  double refl_line_max_length_{3.00};

  /* ===== 反光条高度限制 ===== */
  double refl_z_min_{-0.50};
  double refl_z_max_{ 1.50};

  /* ===== 算法参数（per_ring 模式） ===== */
  double bearing_std_dev_{0.001};
  double range_std_dev_{0.02};
  double ls_angle_thresh_{0.0001};
  double ls_radius_thresh_{0.0001};
  double max_line_gap_{0.05};
  double min_line_length_{0.25};
  int    min_line_points_{8};
  double min_split_dist_{0.04};
  double outlier_dist_{0.06};

  /* ===== 性能参数 ===== */
  double processing_rate_{10.0};
  int    max_points_per_ring_{1800};
  int    cloud_queue_size_{3};

  /* ===== 调试点云发布 ===== */
  std::string refl_cloud_topic_;
  bool        publish_refl_cloud_{true};

  /* ===== 复用缓冲 ===== */
  static constexpr std::size_t MAX_RING = 256;
  std::array<std::vector<Point3D>, MAX_RING> ring_pts_;

  std::vector<double>       bearings_;
  std::vector<double>       cos_bearings_;
  std::vector<double>       sin_bearings_;
  std::vector<double>       ranges_;
  std::vector<unsigned int> indices_;

  bool extractor_configured_{false};
  rclcpp::Time last_process_time_;

  /* ===== ROS 接口 ===== */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<msg::LineSegment3DArray>::SharedPtr lines_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr refl_cloud_pub_;

  line_extraction::LineExtraction extractor_;
};

}  // namespace tanway_line_extractor

#endif