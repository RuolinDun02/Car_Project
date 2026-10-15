cmake_minimum_required(VERSION 3.8)

project(laser_line_extraction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineSegment.msg"
  "msg/LineSegmentList.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(line src/line.cpp)
target_include_directories(line PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(line_extraction src/line_extraction.cpp)
target_include_directories(line_extraction PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(line_extraction line)

add_library(line_extraction_ros src/line_extraction_ros.cpp)
target_include_directories(line_extraction_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(line_extraction_ros
  line
  line_extraction
  "${cpp_typesupport_target}"
)
ament_target_dependencies(line_extraction_ros
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
)

add_executable(line_extraction_node src/line_extraction_node.cpp)
target_link_libraries(line_extraction_node line_extraction_ros)
ament_target_dependencies(line_extraction_node rclcpp)

install(TARGETS line_extraction_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS line_extraction_ros line_extraction line
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

ament_package()
