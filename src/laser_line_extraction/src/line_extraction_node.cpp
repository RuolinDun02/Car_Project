#include "laser_line_extraction/line_extraction_ros.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<line_extraction::LineExtractionROS>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
