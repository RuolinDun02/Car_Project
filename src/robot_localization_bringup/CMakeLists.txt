cmake_minimum_required(VERSION 3.8)
project(robot_localization_bringup)

# 本包只安装 launch 和 config 文件，不编译 C++ 代码，
# 因此不需要 find_package(rclcpp) 等编译依赖。

find_package(ament_cmake REQUIRED)

# 安装 launch 和 config 目录到 share/robot_localization_bringup
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 如果需要运行测试，可保留以下内容；否则可直接删除
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()