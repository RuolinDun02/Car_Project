cmake_minimum_required(VERSION 3.8)

project(tanway_line_extractor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- laser_line_extraction 源码 ----
# 该包没有 ament_export_targets，所以直接编译源码最可靠
set(LLE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../laser_line_extraction")
set(LLE_SRC
  "${LLE_ROOT}/src/line.cpp"
  "${LLE_ROOT}/src/line_extraction.cpp"
)

# ---- 自定义消息 ----
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineSegment3D.msg"
  "msg/LineSegment3DArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ---- 可执行文件 ----
add_executable(tanway_line_extractor_node
  src/tanway_line_extractor_node.cpp
  ${LLE_SRC}
)

target_include_directories(tanway_line_extractor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "${LLE_ROOT}/include"
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(tanway_line_extractor_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  visualization_msgs
)

# Humble 推荐的链接方式
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tanway_line_extractor_node "${cpp_typesupport_target}")

# ---- 安装 ----
install(TARGETS tanway_line_extractor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
