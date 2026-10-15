#include "laser_line_extraction/line_extraction_ros.h"
#include <cmath>

namespace line_extraction
{

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
LineExtractionROS::LineExtractionROS(const rclcpp::NodeOptions & options)
: rclcpp::Node("line_extraction_node", options),
  data_cached_(false)
{
  loadParameters();
  line_publisher_ = create_publisher<laser_line_extraction::msg::LineSegmentList>(
    "line_segments", 1);
  scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, 1,
    std::bind(&LineExtractionROS::laserScanCallback, this, std::placeholders::_1));
  if (pub_markers_)
  {
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>("line_markers", 1);
  }

  double frequency;
  frequency = this->declare_parameter<double>("frequency", 25.0);
  RCLCPP_DEBUG(get_logger(), "Frequency set to %0.1f Hz", frequency);

  auto period = std::chrono::duration<double>(1.0 / frequency);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&LineExtractionROS::run, this));
}

LineExtractionROS::~LineExtractionROS()
{
}

///////////////////////////////////////////////////////////////////////////////
// Run
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::run()
{
  if (!data_cached_)
  {
    return;
  }

  // Extract the lines
  std::vector<Line> lines;
  line_extraction_.extractLines(lines);

  // Populate message
  laser_line_extraction::msg::LineSegmentList msg;
  populateLineSegListMsg(lines, msg);

  // Publish the lines
  line_publisher_->publish(msg);

  // Also publish markers if parameter publish_markers is set to true
  if (pub_markers_)
  {
    visualization_msgs::msg::Marker marker_msg;
    populateMarkerMsg(lines, marker_msg);
    marker_publisher_->publish(marker_msg);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Load ROS parameters
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::loadParameters()
{
  RCLCPP_DEBUG(get_logger(), "*************************************");
  RCLCPP_DEBUG(get_logger(), "PARAMETERS:");

  // Parameters used by this node

  frame_id_ = declare_parameter<std::string>("frame_id", "laser");
  RCLCPP_DEBUG(get_logger(), "frame_id: %s", frame_id_.c_str());

  scan_topic_ = declare_parameter<std::string>("scan_topic", "scan");
  RCLCPP_DEBUG(get_logger(), "scan_topic: %s", scan_topic_.c_str());

  pub_markers_ = declare_parameter<bool>("publish_markers", false);
  RCLCPP_DEBUG(get_logger(), "publish_markers: %s", pub_markers_ ? "true" : "false");

  // Parameters used by the line extraction algorithm

  double bearing_std_dev = declare_parameter<double>("bearing_std_dev", 1e-3);
  line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);
  RCLCPP_DEBUG(get_logger(), "bearing_std_dev: %f", bearing_std_dev);

  double range_std_dev = declare_parameter<double>("range_std_dev", 0.02);
  line_extraction_.setRangeVariance(range_std_dev * range_std_dev);
  RCLCPP_DEBUG(get_logger(), "range_std_dev: %f", range_std_dev);

  double least_sq_angle_thresh = declare_parameter<double>("least_sq_angle_thresh", 1e-4);
  line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
  RCLCPP_DEBUG(get_logger(), "least_sq_angle_thresh: %f", least_sq_angle_thresh);

  double least_sq_radius_thresh = declare_parameter<double>("least_sq_radius_thresh", 1e-4);
  line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
  RCLCPP_DEBUG(get_logger(), "least_sq_radius_thresh: %f", least_sq_radius_thresh);

  double max_line_gap = declare_parameter<double>("max_line_gap", 0.4);
  line_extraction_.setMaxLineGap(max_line_gap);
  RCLCPP_DEBUG(get_logger(), "max_line_gap: %f", max_line_gap);

  double min_line_length = declare_parameter<double>("min_line_length", 0.5);
  line_extraction_.setMinLineLength(min_line_length);
  RCLCPP_DEBUG(get_logger(), "min_line_length: %f", min_line_length);

  double min_range = declare_parameter<double>("min_range", 0.4);
  line_extraction_.setMinRange(min_range);
  RCLCPP_DEBUG(get_logger(), "min_range: %f", min_range);

  double max_range = declare_parameter<double>("max_range", 10000.0);
  line_extraction_.setMaxRange(max_range);
  RCLCPP_DEBUG(get_logger(), "max_range: %f", max_range);

  double min_split_dist = declare_parameter<double>("min_split_dist", 0.05);
  line_extraction_.setMinSplitDist(min_split_dist);
  RCLCPP_DEBUG(get_logger(), "min_split_dist: %f", min_split_dist);

  double outlier_dist = declare_parameter<double>("outlier_dist", 0.05);
  line_extraction_.setOutlierDist(outlier_dist);
  RCLCPP_DEBUG(get_logger(), "outlier_dist: %f", outlier_dist);

  int min_line_points = declare_parameter<int>("min_line_points", 9);
  line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
  RCLCPP_DEBUG(get_logger(), "min_line_points: %d", min_line_points);

  RCLCPP_DEBUG(get_logger(), "*************************************");
}

///////////////////////////////////////////////////////////////////////////////
// Populate messages
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::populateLineSegListMsg(
  const std::vector<Line> & lines,
  laser_line_extraction::msg::LineSegmentList & line_list_msg)
{
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    laser_line_extraction::msg::LineSegment line_msg;
    line_msg.angle = static_cast<float>(cit->getAngle());
    line_msg.radius = static_cast<float>(cit->getRadius());
    for (std::size_t i = 0; i < 4; ++i)
    {
      line_msg.covariance[i] = static_cast<float>(cit->getCovariance()[i]);
    }
    for (std::size_t i = 0; i < 2; ++i)
    {
      line_msg.start[i] = static_cast<float>(cit->getStart()[i]);
      line_msg.end[i] = static_cast<float>(cit->getEnd()[i]);
    }
    line_list_msg.line_segments.push_back(line_msg);
  }
  line_list_msg.header.frame_id = frame_id_;
  line_list_msg.header.stamp = this->now();
}

void LineExtractionROS::populateMarkerMsg(
  const std::vector<Line> & lines,
  visualization_msgs::msg::Marker & marker_msg)
{
  marker_msg.ns = "line_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    geometry_msgs::msg::Point p_start;
    p_start.x = cit->getStart()[0];
    p_start.y = cit->getStart()[1];
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::msg::Point p_end;
    p_end.x = cit->getEnd()[0];
    p_end.y = cit->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = frame_id_;
  marker_msg.header.stamp = this->now();
}

///////////////////////////////////////////////////////////////////////////////
// Cache data on first LaserScan message received
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::cacheData(const sensor_msgs::msg::LaserScan::SharedPtr & scan_msg)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  const std::size_t num_measurements = static_cast<std::size_t>(std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment));
  for (std::size_t i = 0; i < num_measurements; ++i)
  {
    const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(static_cast<unsigned int>(i));
  }

  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  RCLCPP_DEBUG(get_logger(), "Data has been cached.");
}

///////////////////////////////////////////////////////////////////////////////
// Main LaserScan callback
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  if (!data_cached_)
  {
    cacheData(scan_msg);
    data_cached_ = true;
  }

  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
  line_extraction_.setRangeData(scan_ranges_doubles);
}

} // namespace line_extraction
