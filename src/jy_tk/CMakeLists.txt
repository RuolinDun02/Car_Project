cmake_minimum_required(VERSION 3.8)
project(jy_tk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ============================================================
# 安装 Python 脚本到 lib/jy_tk/
# 这样 ros2 run jy_tk nav_trigger.py 就能找到
# ============================================================
install(PROGRAMS
  scripts/nav_trigger.py
  scripts/motor_watchdog.py
  scripts/multi_point_patrol.py
  scripts/arm_nav_bridge.py
  scripts/docking_controller.py
  scripts/polygon_keepout_mask_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# 安装资源文件
# ============================================================
install(DIRECTORY
  urdf
  launch
  config
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()