#include "tanway_line_extractor/tanway_line_extractor.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <unordered_map>

#include <Eigen/Dense>

namespace tanway_line_extractor
{

/* =========================================================
 * 3D 聚类辅助：网格哈希键
 * ========================================================= */
namespace
{
struct GridKey
{
  int32_t ix, iy, iz;
  bool operator==(const GridKey & o) const
  { return ix == o.ix && iy == o.iy && iz == o.iz; }
};

struct GridKeyHash
{
  std::size_t operator()(const GridKey & k) const
  {
    std::size_t h = 1469598103934665603ULL;
    h = (h ^ static_cast<std::size_t>(k.ix)) * 1099511628211ULL;
    h = (h ^ static_cast<std::size_t>(k.iy)) * 1099511628211ULL;
    h = (h ^ static_cast<std::size_t>(k.iz)) * 1099511628211ULL;
    return h;
  }
};
}  // namespace

/* =========================================================
 * 构造函数
 * ========================================================= */
TanwayLineExtractor::TanwayLineExtractor()
: Node("tanway_line_extractor")
{
  /* ---- 话题与坐标系 ---- */
  cloud_topic_   = declare_parameter<std::string>("cloud_topic",   "/tanwaylidar_pointcloud");
  lines_topic_   = declare_parameter<std::string>("lines_topic",   "/tanway_lines");
  markers_topic_ = declare_parameter<std::string>("markers_topic", "/tanway_line_markers");
  frame_id_      = declare_parameter<std::string>("frame_id",      "lidar_link");

  /* ---- 调试点云发布 ---- */
  refl_cloud_topic_   = declare_parameter<std::string>("refl_cloud_topic",
                                                        "/tanway_reflective_points");
  publish_refl_cloud_ = declare_parameter<bool>("publish_refl_cloud", true);

  /* ---- 提取模式 ---- */
  extraction_mode_ = declare_parameter<std::string>("extraction_mode", "3d_cluster");

  /* ---- 点云过滤 ---- */
  min_range_     = declare_parameter<double>("min_range",     0.3);
  max_range_     = declare_parameter<double>("max_range",     30.0);
  min_intensity_ = declare_parameter<int>   ("min_intensity", 0);
  max_intensity_ = declare_parameter<int>   ("max_intensity", 1000);
  ring_min_      = declare_parameter<int>   ("ring_min",      1);
  ring_max_      = declare_parameter<int>   ("ring_max",      48);

  /* ---- 3D 聚类模式参数 ---- */
  reflective_threshold_  = declare_parameter<double>("reflective_threshold",  180.0);
  cluster_eps_           = declare_parameter<double>("cluster_eps",           0.08);
  cluster_min_pts_       = declare_parameter<int>   ("cluster_min_pts",       15);
  linearity_threshold_   = declare_parameter<double>("linearity_threshold",   8.0);

  /* ---- 反光条长度限制 ---- */
  refl_line_min_length_ = declare_parameter<double>("reflective_line_min_length", 0.20);
  refl_line_max_length_ = declare_parameter<double>("reflective_line_max_length", 3.00);

  /* ---- 反光条高度限制 ---- */
  refl_z_min_ = declare_parameter<double>("reflective_z_min", -0.50);
  refl_z_max_ = declare_parameter<double>("reflective_z_max",  1.50);

  /* ---- 算法参数（per_ring 模式） ---- */
  bearing_std_dev_  = declare_parameter<double>("bearing_std_dev",        0.001);
  range_std_dev_    = declare_parameter<double>("range_std_dev",          0.02);
  ls_angle_thresh_  = declare_parameter<double>("least_sq_angle_thresh",  0.0001);
  ls_radius_thresh_ = declare_parameter<double>("least_sq_radius_thresh", 0.0001);
  max_line_gap_     = declare_parameter<double>("max_line_gap",           0.05);
  min_line_length_  = declare_parameter<double>("min_line_length",        0.25);
  min_line_points_  = declare_parameter<int>   ("min_line_points",        8);
  min_split_dist_   = declare_parameter<double>("min_split_dist",         0.04);
  outlier_dist_     = declare_parameter<double>("outlier_dist",           0.06);

  /* ---- 性能参数 ---- */
  processing_rate_     = declare_parameter<double>("processing_rate",     10.0);
  max_points_per_ring_ = declare_parameter<int>   ("max_points_per_ring", 1800);
  cloud_queue_size_    = declare_parameter<int>   ("cloud_queue_size",    3);

  /* =====================================================
   * 参数合法性检查
   * ===================================================== */
  if (extraction_mode_ != "per_ring" && extraction_mode_ != "3d_cluster") {
    RCLCPP_WARN(get_logger(), "未知 extraction_mode='%s'，回退到 per_ring",
      extraction_mode_.c_str());
    extraction_mode_ = "per_ring";
  }
  if (processing_rate_ < 0.0)      processing_rate_ = 0.0;
  if (max_points_per_ring_ < 0)    max_points_per_ring_ = 0;
  if (cloud_queue_size_ < 1)       cloud_queue_size_ = 1;
  if (ring_min_ < 0)               ring_min_ = 0;
  if (ring_max_ >= static_cast<int>(MAX_RING)) ring_max_ = static_cast<int>(MAX_RING) - 1;
  if (cluster_eps_ < 0.01)         cluster_eps_ = 0.01;
  if (cluster_min_pts_ < 3)        cluster_min_pts_ = 3;
  if (refl_line_min_length_ < 0.0) refl_line_min_length_ = 0.0;
  if (refl_line_max_length_ <= refl_line_min_length_) {
    refl_line_max_length_ = refl_line_min_length_ + 0.01;
  }
  if (refl_z_min_ > refl_z_max_) {
    std::swap(refl_z_min_, refl_z_max_);
  }

  /* =====================================================
   * 创建 ROS 接口
   * ===================================================== */
  lines_pub_   = create_publisher<msg::LineSegment3DArray>(lines_topic_, 10);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, 10);

  /* ---- 调试点云发布者 ---- */
  refl_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    refl_cloud_topic_, 10);

  auto qos = rclcpp::SensorDataQoS().keep_last(cloud_queue_size_);
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, qos,
    std::bind(&TanwayLineExtractor::cloudCallback, this, std::placeholders::_1));

  const std::size_t reserve_per_ring = 512;
  for (int r = ring_min_; r <= ring_max_; ++r) {
    ring_pts_[r].reserve(reserve_per_ring);
  }

  configureExtractor(extractor_);
  extractor_configured_ = true;

  last_process_time_ = now();

  RCLCPP_INFO(get_logger(),
    "启动: 订阅=%s | 模式=%s | refl_thr=%.1f eps=%.3f "
    "min_pts=%d linearity_thr=%.1f len=[%.2f,%.2f] z=[%.2f,%.2f]",
    cloud_topic_.c_str(), extraction_mode_.c_str(),
    reflective_threshold_, cluster_eps_, cluster_min_pts_,
    linearity_threshold_,
    refl_line_min_length_, refl_line_max_length_,
    refl_z_min_, refl_z_max_);

  if (publish_refl_cloud_) {
    RCLCPP_INFO(get_logger(), "高反光点云发布: %s (调试用)",
      refl_cloud_topic_.c_str());
  }
}

/* =========================================================
 * 配置 LineExtraction 参数
 * ========================================================= */
void TanwayLineExtractor::configureExtractor(
  line_extraction::LineExtraction & ex) const
{
  ex.setBearingVariance(bearing_std_dev_ * bearing_std_dev_);
  ex.setRangeVariance(range_std_dev_ * range_std_dev_);
  ex.setLeastSqAngleThresh(ls_angle_thresh_);
  ex.setLeastSqRadiusThresh(ls_radius_thresh_);
  ex.setMaxLineGap(max_line_gap_);
  ex.setMinLineLength(min_line_length_);
  ex.setMinLinePoints(static_cast<unsigned int>(min_line_points_));
  ex.setMinRange(min_range_);
  ex.setMaxRange(max_range_);
  ex.setMinSplitDist(min_split_dist_);
  ex.setOutlierDist(outlier_dist_);
}

/* =========================================================
 * 发布高反光点云（调试）
 * ========================================================= */
void TanwayLineExtractor::publishReflectiveCloud(
  const std::vector<Point3D> & pts,
  const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  sensor_msgs::msg::PointCloud2 cloud;

  cloud.header.stamp    = stamp;
  cloud.header.frame_id = frame_id;

  cloud.height       = 1;
  cloud.width        = static_cast<uint32_t>(pts.size());
  cloud.is_bigendian = false;
  cloud.is_dense     = true;
  cloud.point_step   = 16;
  cloud.row_step     = cloud.point_step * cloud.width;

  cloud.data.resize(cloud.row_step);

  uint8_t * ptr = cloud.data.data();
  for (const auto & p : pts) {
    std::memcpy(ptr + 0,  &p.x,         4);
    std::memcpy(ptr + 4,  &p.y,         4);
    std::memcpy(ptr + 8,  &p.z,         4);
    std::memcpy(ptr + 12, &p.intensity, 4);
    ptr += 16;
  }

  cloud.fields.resize(4);

  cloud.fields[0].name     = "x";
  cloud.fields[0].offset   = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count    = 1;

  cloud.fields[1].name     = "y";
  cloud.fields[1].offset   = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count    = 1;

  cloud.fields[2].name     = "z";
  cloud.fields[2].offset   = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count    = 1;

  cloud.fields[3].name     = "intensity";
  cloud.fields[3].offset   = 12;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].count    = 1;

  refl_cloud_pub_->publish(cloud);
}

/* =========================================================
 * 点云回调
 * ========================================================= */
void TanwayLineExtractor::cloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (processing_rate_ > 0.0) {
    const rclcpp::Time now_t = now();
    if ((now_t - last_process_time_).seconds() < 1.0 / processing_rate_) {
      return;
    }
    last_process_time_ = now_t;
  }

  const rclcpp::Time stamp = msg->header.stamp;

  auto has_field = [&msg](const std::string & name) {
    for (const auto & f : msg->fields) if (f.name == name) return true;
    return false;
  };

  if (!has_field("x") || !has_field("y") || !has_field("z")) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "点云缺少 x/y/z 字段");
    return;
  }

  msg::LineSegment3DArray out;
  out.header.stamp    = stamp;
  out.header.frame_id = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;

  visualization_msgs::msg::MarkerArray markers;

  if (extraction_mode_ == "3d_cluster") {
    extract3DCluster(msg, out, markers);
  } else {
    extractPerRing(msg, out, markers);
  }

  lines_pub_->publish(out);
  markers_pub_->publish(markers);

  RCLCPP_DEBUG(get_logger(), "模式=%s 线段=%zu",
    extraction_mode_.c_str(), out.lines.size());
}

/* =========================================================
 * 模式 A：逐层提取
 * ========================================================= */
void TanwayLineExtractor::extractPerRing(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  msg::LineSegment3DArray & out,
  visualization_msgs::msg::MarkerArray & markers)
{
  auto has_field = [&msg](const std::string & name) {
    for (const auto & f : msg->fields) if (f.name == name) return true;
    return false;
  };

  if (!has_field("channel")) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "per_ring 模式需要 channel 字段");
    return;
  }
  const bool has_intensity = has_field("intensity");
  const rclcpp::Time stamp = msg->header.stamp;

  for (int r = ring_min_; r <= ring_max_; ++r) {
    ring_pts_[r].clear();
  }

  const float min_r2 = static_cast<float>(min_range_ * min_range_);
  const float max_r2 = static_cast<float>(max_range_ * max_range_);
  const float min_i  = static_cast<float>(min_intensity_);
  const float max_i  = static_cast<float>(max_intensity_);
  const uint16_t ring_lo = static_cast<uint16_t>(ring_min_);
  const uint16_t ring_hi = static_cast<uint16_t>(ring_max_);

  try {
    sensor_msgs::PointCloud2ConstIterator<float>    it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float>    it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float>    it_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> it_channel(*msg, "channel");

    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_int;
    if (has_intensity) {
      it_int = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*msg, "intensity");
    }

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_channel) {
      const float    x    = *it_x;
      const float    y    = *it_y;
      const float    z    = *it_z;
      const uint16_t ring = *it_channel;

      if (ring < ring_lo || ring > ring_hi) {
        if (it_int) ++(*it_int);
        continue;
      }

      const float r2 = x * x + y * y;
      if (r2 < min_r2 || r2 > max_r2) {
        if (it_int) ++(*it_int);
        continue;
      }

      float inten = 0.0f;
      if (it_int) {
        inten = **it_int;
        ++(*it_int);
        if (inten < min_i || inten > max_i) continue;
      }

      Point3D p;
      p.x = x;
      p.y = y;
      p.z = z;
      p.intensity = inten;
      ring_pts_[ring].push_back(p);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
      "解析点云失败: %s", e.what());
    return;
  }

  markers.markers.reserve(static_cast<std::size_t>(ring_max_ - ring_min_ + 1));
  int marker_id = 0;

  for (int r = ring_min_; r <= ring_max_; ++r) {
    auto & pts = ring_pts_[r];
    if (pts.size() < static_cast<std::size_t>(min_line_points_)) continue;

    if (max_points_per_ring_ > 0 &&
        pts.size() > static_cast<std::size_t>(max_points_per_ring_))
    {
      const std::size_t stride =
        pts.size() / static_cast<std::size_t>(max_points_per_ring_) + 1;
      std::size_t j = 0;
      for (std::size_t i = 0; i < pts.size(); i += stride) {
        pts[j++] = pts[i];
      }
      pts.resize(j);
    }

    extractFromRing(static_cast<uint16_t>(r), pts, stamp, out, markers, marker_id);
  }
}

/* =========================================================
 * per_ring 模式的单层提取
 * ========================================================= */
void TanwayLineExtractor::extractFromRing(
  uint16_t ring,
  std::vector<Point3D> & pts,
  const rclcpp::Time & stamp,
  msg::LineSegment3DArray & out,
  visualization_msgs::msg::MarkerArray & markers,
  int & marker_id)
{
  std::sort(pts.begin(), pts.end(),
    [](const Point3D & a, const Point3D & b) {
      return std::atan2(a.y, a.x) < std::atan2(b.y, b.x);
    });

  const std::size_t n = pts.size();

  bearings_.clear();     cos_bearings_.clear();
  sin_bearings_.clear(); ranges_.clear();
  indices_.clear();
  bearings_.reserve(n);     cos_bearings_.reserve(n);
  sin_bearings_.reserve(n); ranges_.reserve(n);
  indices_.reserve(n);

  double sum_z = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double b = std::atan2(static_cast<double>(pts[i].y),
                                 static_cast<double>(pts[i].x));
    const double r = std::sqrt(static_cast<double>(pts[i].x) * pts[i].x +
                                static_cast<double>(pts[i].y) * pts[i].y);
    bearings_.push_back(b);
    cos_bearings_.push_back(std::cos(b));
    sin_bearings_.push_back(std::sin(b));
    ranges_.push_back(r);
    indices_.push_back(static_cast<unsigned int>(i));
    sum_z += pts[i].z;
  }
  const double avg_z = sum_z / static_cast<double>(n);

  if (!extractor_configured_) {
    configureExtractor(extractor_);
    extractor_configured_ = true;
  }
  extractor_.setCachedData(bearings_, cos_bearings_, sin_bearings_, indices_);
  extractor_.setRangeData(ranges_);

  std::vector<line_extraction::Line> lines;
  extractor_.extractLines(lines);

  for (const auto & line : lines) {
    const auto & start = line.getStart();
    const auto & end   = line.getEnd();

    const double dx  = end[0] - start[0];
    const double dy  = end[1] - start[1];
    const double len = std::sqrt(dx * dx + dy * dy);

    if (len < min_line_length_) continue;

    float avg_intensity = 0.0f;
    const auto & idxs = line.getIndices();
    if (!idxs.empty()) {
      double sum_i = 0.0;
      std::size_t cnt = 0;
      for (auto idx : idxs) {
        if (idx < pts.size()) { sum_i += pts[idx].intensity; ++cnt; }
      }
      if (cnt > 0) avg_intensity = static_cast<float>(sum_i / static_cast<double>(cnt));
    }

    msg::LineSegment3D lm;
    lm.header.stamp    = stamp;
    lm.header.frame_id = out.header.frame_id;
    lm.start.x = static_cast<float>(start[0]);
    lm.start.y = static_cast<float>(start[1]);
    lm.start.z = static_cast<float>(avg_z);
    lm.end.x   = static_cast<float>(end[0]);
    lm.end.y   = static_cast<float>(end[1]);
    lm.end.z   = static_cast<float>(avg_z);
    lm.midpoint.x = static_cast<float>(0.5 * (start[0] + end[0]));
    lm.midpoint.y = static_cast<float>(0.5 * (start[1] + end[1]));
    lm.midpoint.z = static_cast<float>(avg_z);
    lm.length        = static_cast<float>(len);
    lm.bearing_start = static_cast<float>(std::atan2(start[1], start[0]));
    lm.bearing_end   = static_cast<float>(std::atan2(end[1],   end[0]));
    lm.ring          = ring;
    lm.num_points    = static_cast<uint16_t>(idxs.size());
    lm.avg_intensity = avg_intensity;

    out.lines.push_back(lm);

    visualization_msgs::msg::Marker m;
    m.header.stamp    = stamp;
    m.header.frame_id = out.header.frame_id;
    m.ns     = "tanway_lines";
    m.id     = marker_id++;
    m.type   = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.02;
    m.pose.orientation.w = 1.0;
    m.color.a = 1.0f;
    m.color.r = static_cast<float>(ring) / 48.0f;
    m.color.g = 1.0f - static_cast<float>(ring) / 48.0f;
    m.color.b = 0.3f;

    m.points.reserve(2);
    m.points.push_back(lm.start);
    m.points.push_back(lm.end);
    markers.markers.push_back(m);
  }
}

/* =========================================================
 * 模式 B：3D 聚类 + PCA 直线拟合
 * ========================================================= */
void TanwayLineExtractor::extract3DCluster(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  msg::LineSegment3DArray & out,
  visualization_msgs::msg::MarkerArray & markers)
{
  const rclcpp::Time stamp = msg->header.stamp;

  auto has_field = [&msg](const std::string & name) {
    for (const auto & f : msg->fields) if (f.name == name) return true;
    return false;
  };

  if (!has_field("intensity")) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "3d_cluster 模式需要 intensity 字段");
    return;
  }

  static int frame_counter = 0;
  ++frame_counter;
  const bool diag_on = (frame_counter % 30 == 1);

  /* =====================================================
   * 1. 收集高反光点
   * ===================================================== */
  std::vector<Point3D> refl_pts;
  refl_pts.reserve(4096);

  float max_intensity_seen = 0.0f;
  float min_z_seen = std::numeric_limits<float>::max();
  float max_z_seen = std::numeric_limits<float>::lowest();
  std::size_t total_pts = 0;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_int(*msg, "intensity");

    const float min_r2   = static_cast<float>(min_range_ * min_range_);
    const float max_r2   = static_cast<float>(max_range_ * max_range_);
    const float refl_thr = static_cast<float>(reflective_threshold_);
    const float z_lo     = static_cast<float>(refl_z_min_);
    const float z_hi     = static_cast<float>(refl_z_max_);

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_int) {
      ++total_pts;

      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;
      const float inten = *it_int;

      if (inten > max_intensity_seen) max_intensity_seen = inten;
      if (z < min_z_seen) min_z_seen = z;
      if (z > max_z_seen) max_z_seen = z;

      if (inten < refl_thr) continue;

      const float r2 = x * x + y * y;
      if (r2 < min_r2 || r2 > max_r2) continue;

      if (z < z_lo || z > z_hi) continue;

      refl_pts.push_back({x, y, z, inten});
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
      "3d_cluster 点云解析失败: %s", e.what());
    return;
  }

  /* ★ 发布高反光点云（调试） */
  if (publish_refl_cloud_) {
    publishReflectiveCloud(refl_pts, stamp, out.header.frame_id);
  }

  if (diag_on) {
    RCLCPP_INFO(get_logger(),
      "\n========== [诊断帧 %d] ==========\n"
      "点云总点数       = %zu\n"
      "点云最大强度     = %.1f (阈值=%.1f)\n"
      "点云 z 范围      = [%.3f, %.3f] (过滤范围=[%.2f, %.2f])\n"
      "高反光点数       = %zu (参数 cluster_min_pts=%d)",
      frame_counter, total_pts,
      max_intensity_seen, reflective_threshold_,
      min_z_seen, max_z_seen, refl_z_min_, refl_z_max_,
      refl_pts.size(), cluster_min_pts_);
  }

  if (refl_pts.size() < static_cast<std::size_t>(cluster_min_pts_)) {
    if (diag_on) {
      RCLCPP_WARN(get_logger(),
        "[诊断] 高反光点不足 (%zu < %d)，跳过。",
        refl_pts.size(), cluster_min_pts_);
    }
    return;
  }

  /* =====================================================
   * 2. 3D 欧式聚类
   * ===================================================== */
  const double inv_eps = 1.0 / cluster_eps_;
  std::unordered_map<GridKey, std::vector<std::size_t>, GridKeyHash> grid;
  grid.reserve(refl_pts.size() * 2);

  auto to_key = [inv_eps](float x, float y, float z) -> GridKey {
    return GridKey{
      static_cast<int32_t>(std::floor(x * inv_eps)),
      static_cast<int32_t>(std::floor(y * inv_eps)),
      static_cast<int32_t>(std::floor(z * inv_eps))
    };
  };

  for (std::size_t i = 0; i < refl_pts.size(); ++i) {
    grid[to_key(refl_pts[i].x, refl_pts[i].y, refl_pts[i].z)].push_back(i);
  }

  const double eps2 = cluster_eps_ * cluster_eps_;
  std::vector<bool> visited(refl_pts.size(), false);
  std::vector<std::vector<std::size_t>> clusters;

  for (std::size_t seed = 0; seed < refl_pts.size(); ++seed) {
    if (visited[seed]) continue;

    std::vector<std::size_t> cluster;
    std::queue<std::size_t> q;
    q.push(seed);
    visited[seed] = true;

    while (!q.empty()) {
      const std::size_t cur = q.front();
      q.pop();
      cluster.push_back(cur);

      const GridKey kc = to_key(refl_pts[cur].x, refl_pts[cur].y, refl_pts[cur].z);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            const GridKey kn{kc.ix + dx, kc.iy + dy, kc.iz + dz};
            auto it = grid.find(kn);
            if (it == grid.end()) continue;

            for (std::size_t j : it->second) {
              if (visited[j]) continue;
              const float ddx = refl_pts[j].x - refl_pts[cur].x;
              const float ddy = refl_pts[j].y - refl_pts[cur].y;
              const float ddz = refl_pts[j].z - refl_pts[cur].z;
              if (ddx * ddx + ddy * ddy + ddz * ddz <= eps2) {
                visited[j] = true;
                q.push(j);
              }
            }
          }
        }
      }
    }

    if (static_cast<int>(cluster.size()) >= cluster_min_pts_) {
      clusters.push_back(std::move(cluster));
    }
  }

  if (diag_on) {
    RCLCPP_INFO(get_logger(),
      "[诊断] 3D 聚类完成: 有效 cluster 数=%zu (eps=%.3f, min_pts=%d)",
      clusters.size(), cluster_eps_, cluster_min_pts_);
  }

  /* =====================================================
   * 3. PCA 拟合 + 过滤
   * ===================================================== */
  markers.markers.reserve(clusters.size());
  int marker_id = 0;

  int filtered_by_length = 0;
  int filtered_by_z      = 0;
  int filtered_by_linear = 0;

  for (const auto & cluster : clusters) {
    const std::size_t n = cluster.size();
    if (n < 3) continue;

    Eigen::Vector3d centroid(0, 0, 0);
    for (auto idx : cluster) {
      centroid += Eigen::Vector3d(refl_pts[idx].x, refl_pts[idx].y, refl_pts[idx].z);
    }
    centroid /= static_cast<double>(n);

    if (centroid.z() < refl_z_min_ || centroid.z() > refl_z_max_) {
      ++filtered_by_z;
      continue;
    }

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (auto idx : cluster) {
      const Eigen::Vector3d d = Eigen::Vector3d(
        refl_pts[idx].x, refl_pts[idx].y, refl_pts[idx].z) - centroid;
      cov += d * d.transpose();
    }
    cov /= static_cast<double>(n);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    if (solver.info() != Eigen::Success) continue;

    const double lam_min = solver.eigenvalues()(0);
    const double lam_mid = solver.eigenvalues()(1);
    const double lam_max = solver.eigenvalues()(2);
    (void)lam_min;

    if (lam_mid < 1e-9) continue;

    const double linearity = lam_max / lam_mid;
    const Eigen::Vector3d dir = solver.eigenvectors().col(2).normalized();

    double t_min = std::numeric_limits<double>::max();
    double t_max = std::numeric_limits<double>::lowest();
    for (auto idx : cluster) {
      const Eigen::Vector3d p(refl_pts[idx].x, refl_pts[idx].y, refl_pts[idx].z);
      const double t = dir.dot(p - centroid);
      t_min = std::min(t_min, t);
      t_max = std::max(t_max, t);
    }
    const double length = t_max - t_min;

    if (diag_on) {
      RCLCPP_INFO(get_logger(),
        "[诊断] cluster: n=%zu z=%.3f 长度=%.3f linearity=%.2f",
        n, centroid.z(), length, linearity);
    }

    if (linearity < linearity_threshold_) {
      ++filtered_by_linear;
      continue;
    }

    if (length < refl_line_min_length_ || length > refl_line_max_length_) {
      ++filtered_by_length;
      continue;
    }

    const Eigen::Vector3d p_start = centroid + t_min * dir;
    const Eigen::Vector3d p_end   = centroid + t_max * dir;

    double sum_i = 0.0;
    for (auto idx : cluster) sum_i += refl_pts[idx].intensity;
    const float avg_intensity = static_cast<float>(sum_i / static_cast<double>(n));

    msg::LineSegment3D lm;
    lm.header.stamp    = stamp;
    lm.header.frame_id = out.header.frame_id;
    lm.start.x = static_cast<float>(p_start.x());
    lm.start.y = static_cast<float>(p_start.y());
    lm.start.z = static_cast<float>(p_start.z());
    lm.end.x   = static_cast<float>(p_end.x());
    lm.end.y   = static_cast<float>(p_end.y());
    lm.end.z   = static_cast<float>(p_end.z());
    lm.midpoint.x = static_cast<float>(0.5 * (p_start.x() + p_end.x()));
    lm.midpoint.y = static_cast<float>(0.5 * (p_start.y() + p_end.y()));
    lm.midpoint.z = static_cast<float>(0.5 * (p_start.z() + p_end.z()));
    lm.length        = static_cast<float>(length);
    lm.bearing_start = static_cast<float>(std::atan2(p_start.y(), p_start.x()));
    lm.bearing_end   = static_cast<float>(std::atan2(p_end.y(),   p_end.x()));
    lm.ring          = 0;
    lm.num_points    = static_cast<uint16_t>(n);
    lm.avg_intensity = avg_intensity;

    out.lines.push_back(lm);

    visualization_msgs::msg::Marker m;
    m.header.stamp    = stamp;
    m.header.frame_id = out.header.frame_id;
    m.ns     = "tanway_reflective_lines";
    m.id     = marker_id++;
    m.type   = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.03;
    m.pose.orientation.w = 1.0;
    m.color.a = 1.0f;
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.points.reserve(2);
    m.points.push_back(lm.start);
    m.points.push_back(lm.end);
    markers.markers.push_back(m);
  }

  if (diag_on) {
    RCLCPP_INFO(get_logger(),
      "========== [汇总] ==========\n"
      "高反光点: %zu → 有效 cluster: %zu → 输出线段: %zu\n"
      "过滤: 长度不符=%d 高度不符=%d 线性度不足=%d\n"
      "============================",
      refl_pts.size(), clusters.size(), out.lines.size(),
      filtered_by_length, filtered_by_z, filtered_by_linear);
  }
}

}  // namespace tanway_line_extractor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tanway_line_extractor::TanwayLineExtractor>());
  rclcpp::shutdown();
  return 0;
}