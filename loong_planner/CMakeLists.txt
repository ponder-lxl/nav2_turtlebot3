cmake_minimum_required(VERSION 3.8)
project(loong_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
)

add_library(loong_planner SHARED
  src/loong_planner.cpp
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
)

ament_target_dependencies(loong_planner
  ${dependencies}
)

install(TARGETS loong_planner
  LIBRARY DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(loong_planner)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core loong_planner.xml)

ament_package()
