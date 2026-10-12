#include "tanway_line_extractor/tanway_line_extractor.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace tanway_line_extractor
{

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

  /* ---- 点云过滤 ---- */
  min_range_     = declare_parameter<double>("min_range",     0.3);
  max_range_     = declare_parameter<double>("max_range",     30.0);
  min_intensity_ = declare_parameter<int>   ("min_intensity", 0);
  max_intensity_ = declare_parameter<int>   ("max_intensity", 1000);
  ring_min_      = declare_parameter<int>   ("ring_min",      1);
  ring_max_      = declare_parameter<int>   ("ring_max",      48);

  /* ---- 算法参数 ---- */
  bearing_std_dev_  = declare_parameter<double>("bearing_std_dev",        0.001);
  range_std_dev_    = declare_parameter<double>("range_std_dev",          0.02);
  ls_angle_thresh_  = declare_parameter<double>("least_sq_angle_thresh",  0.0001);
  ls_radius_thresh_ = declare_parameter<double>("least_sq_radius_thresh", 0.0001);
  max_line_gap_     = declare_parameter<double>("max_line_gap",           0.05);
  min_line_length_  = declare_parameter<double>("min_line_length",        0.25);
  min_line_points_  = declare_parameter<int>   ("min_line_points",        8);
  min_split_dist_   = declare_parameter<double>("min_split_dist",         0.04);
  outlier_dist_     = declare_parameter<double>("outlier_dist",           0.06);

  /* ---- 性能优化参数 ---- */
  processing_rate_     = declare_parameter<double>("processing_rate",     10.0);
  max_points_per_ring_ = declare_parameter<int>   ("max_points_per_ring", 1800);
  cloud_queue_size_    = declare_parameter<int>   ("cloud_queue_size",    3);

  // 参数合法性
  if (processing_rate_ < 0.0)      processing_rate_ = 0.0;
  if (max_points_per_ring_ < 0)    max_points_per_ring_ = 0;
  if (cloud_queue_size_ < 1)       cloud_queue_size_ = 1;
  if (ring_min_ < 0)               ring_min_ = 0;
  if (ring_max_ >= static_cast<int>(MAX_RING)) ring_max_ = static_cast<int>(MAX_RING) - 1;

  /* ---- 发布者 ---- */
  lines_pub_   = create_publisher<msg::LineSegment3DArray>(lines_topic_, 10);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, 10);

  /* ---- 订阅者：浅队列，减少点云缓冲 ---- */
  auto qos = rclcpp::SensorDataQoS().keep_last(cloud_queue_size_);
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, qos,
    std::bind(&TanwayLineExtractor::cloudCallback, this, std::placeholders::_1));

  /* ---- 预分配 ring_pts_ 容量，避免运行时反复扩容 ---- */
  const std::size_t reserve_per_ring = 512;
  for (int r = ring_min_; r <= ring_max_; ++r) {
    ring_pts_[r].reserve(reserve_per_ring);
  }

  /* ---- 预配置 LineExtraction，避免每层调 10 个 setter ---- */
  configureExtractor(extractor_);
  extractor_configured_ = true;

  /* ---- 初始化时间戳 ---- */
  last_process_time_ = now();

  RCLCPP_INFO(get_logger(),
    "启动: 订阅=%s, 发布=%s + %s, frame_id=%s | "
    "processing_rate=%.1fHz, max_points_per_ring=%d, queue=%d",
    cloud_topic_.c_str(), lines_topic_.c_str(), markers_topic_.c_str(),
    frame_id_.c_str(), processing_rate_, max_points_per_ring_, cloud_queue_size_);
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
 * 点云回调
 * ========================================================= */
void TanwayLineExtractor::cloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  /* ---- 降频：离上次处理不足 1/processing_rate 秒则跳过 ---- */
  if (processing_rate_ > 0.0) {
    const rclcpp::Time now_t = now();
    if ((now_t - last_process_time_).seconds() < 1.0 / processing_rate_) {
      return;
    }
    last_process_time_ = now_t;
  }

  const rclcpp::Time stamp = msg->header.stamp;

  /* ---- 字段检查 ---- */
  auto has_field = [&msg](const std::string & name) {
    for (const auto & f : msg->fields) if (f.name == name) return true;
    return false;
  };

  if (!has_field("x") || !has_field("y") || !has_field("z") || !has_field("channel")) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "点云缺少 x/y/z/channel 字段");
    return;
  }
  const bool has_intensity = has_field("intensity");

  /* ---- 清空上一帧的点，保留 capacity ---- */
  for (int r = ring_min_; r <= ring_max_; ++r) {
    ring_pts_[r].clear();
  }

  /* ---- 预计算范围过滤的平方，避免每点开根号 ---- */
  const float min_r2 = static_cast<float>(min_range_ * min_range_);
  const float max_r2 = static_cast<float>(max_range_ * max_range_);
  const float min_i  = static_cast<float>(min_intensity_);
  const float max_i  = static_cast<float>(max_intensity_);
  const uint16_t ring_lo = static_cast<uint16_t>(ring_min_);
  const uint16_t ring_hi = static_cast<uint16_t>(ring_max_);

  /* ---- 遍历点云，按 channel 分组 ---- */
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

      // 用平方比较代替 hypot，省一次 sqrt
      const float r2 = x * x + y * y;
      if (r2 < min_r2 || r2 > max_r2) {
        if (it_int) ++(*it_int);
        continue;
      }

      // 强度过滤
      float inten = 0.0f;
      if (it_int) {
        inten = **it_int;
        ++(*it_int);
        if (inten < min_i || inten > max_i) continue;
      }

      Point2D p;
      p.x = x;
      p.y = y;
      p.z = z;
      p.range     = std::sqrt(r2);
      p.bearing   = std::atan2(y, x);
      p.intensity = inten;
      ring_pts_[ring].push_back(p);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
      "解析点云失败: %s", e.what());
    return;
  }

  /* ---- 逐层提取 ---- */
  msg::LineSegment3DArray out;
  out.header.stamp    = stamp;
  out.header.frame_id = frame_id_;

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.reserve(static_cast<std::size_t>(ring_max_ - ring_min_ + 1));
  int marker_id = 0;

  for (int r = ring_min_; r <= ring_max_; ++r) {
    auto & pts = ring_pts_[r];
    if (pts.size() < static_cast<std::size_t>(min_line_points_)) continue;

    /* ---- 每层点数超限：原地等间隔下采样（不分配新内存）---- */
    if (max_points_per_ring_ > 0 &&
        pts.size() > static_cast<std::size_t>(max_points_per_ring_))
    {
      const std::size_t stride = pts.size() / static_cast<std::size_t>(max_points_per_ring_) + 1;
      std::size_t j = 0;
      for (std::size_t i = 0; i < pts.size(); i += stride) {
        pts[j++] = pts[i];
      }
      pts.resize(j);
    }

    extractFromRing(static_cast<uint16_t>(r), pts, stamp, out, markers, marker_id);
  }

  lines_pub_->publish(out);
  markers_pub_->publish(markers);

  RCLCPP_DEBUG(get_logger(), "线段=%zu", out.lines.size());
}

/* =========================================================
 * 单层直线提取
 * ========================================================= */
void TanwayLineExtractor::extractFromRing(
  uint16_t ring,
  std::vector<Point2D> & pts,
  const rclcpp::Time & stamp,
  msg::LineSegment3DArray & out,
  visualization_msgs::msg::MarkerArray & markers,
  int & marker_id)
{
  /* ---- 1. 按方位角排序 ---- */
  std::sort(pts.begin(), pts.end(),
    [](const Point2D & a, const Point2D & b) { return a.bearing < b.bearing; });

  const std::size_t n = pts.size();

  /* ---- 2. 复用缓冲区构造 CachedData 输入 ----
   *   clear() 只改 size，不释放 capacity，避免重复 malloc。
   */
  bearings_.clear();
  cos_bearings_.clear();
  sin_bearings_.clear();
  ranges_.clear();
  indices_.clear();
  bearings_.reserve(n);
  cos_bearings_.reserve(n);
  sin_bearings_.reserve(n);
  ranges_.reserve(n);
  indices_.reserve(n);

  double sum_z = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double b = static_cast<double>(pts[i].bearing);
    bearings_.push_back(b);
    cos_bearings_.push_back(std::cos(b));
    sin_bearings_.push_back(std::sin(b));
    ranges_.push_back(static_cast<double>(pts[i].range));
    indices_.push_back(static_cast<unsigned int>(i));
    sum_z += pts[i].z;
  }
  const double avg_z = sum_z / static_cast<double>(n);

  /* ---- 3. 保险：构造时已配置，这里兜底 ---- */
  if (!extractor_configured_) {
    configureExtractor(extractor_);
    extractor_configured_ = true;
  }

  /* ---- 4. 喂入数据 ---- */
  extractor_.setCachedData(bearings_, cos_bearings_, sin_bearings_, indices_);
  extractor_.setRangeData(ranges_);

  /* ---- 5. 执行提取 ---- */
  std::vector<line_extraction::Line> lines;
  extractor_.extractLines(lines);

  /* ---- 6. 遍历结果 ---- */
  for (const auto & line : lines) {
    const auto & start = line.getStart();
    const auto & end   = line.getEnd();

    const double dx  = end[0] - start[0];
    const double dy  = end[1] - start[1];
    const double len = std::sqrt(dx * dx + dy * dy);

    if (len < min_line_length_) continue;

    /* ---- 平均强度 ---- */
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

    /* ---- 填充 LineSegment3D ---- */
    msg::LineSegment3D lm;
    lm.header.stamp    = stamp;
    lm.header.frame_id = frame_id_;

    lm.start.x = static_cast<float>(start[0]);
    lm.start.y = static_cast<float>(start[1]);
    lm.start.z = static_cast<float>(avg_z);

    lm.end.x = static_cast<float>(end[0]);
    lm.end.y = static_cast<float>(end[1]);
    lm.end.z = static_cast<float>(avg_z);

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

    /* ---- Marker ---- */
    visualization_msgs::msg::Marker m;
    m.header.stamp    = stamp;
    m.header.frame_id = frame_id_;
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

}  // namespace tanway_line_extractor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tanway_line_extractor::TanwayLineExtractor>());
  rclcpp::shutdown();
  return 0;
}