cmake_minimum_required(VERSION 3.8)
project(takephoto_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(action_msgs REQUIRED)

include_directories(
  include
)

add_library(bt_take_photo SHARED src/take_photo_action.cpp)
target_include_directories(bt_take_photo PUBLIC include)
ament_target_dependencies(bt_take_photo
    rclcpp
    sensor_msgs
    cv_bridge
    behaviortree_cpp_v3
    OpenCV
    nav2_behavior_tree
)

add_executable(nav_photo_node src/take_photo_node.cpp)

ament_target_dependencies(nav_photo_node
  rclcpp
  sensor_msgs
  action_msgs
  cv_bridge
  OpenCV
  lifecycle_msgs
)

install(TARGETS bt_take_photo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS nav_photo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install BT plugin xml so nav2/behaviortree can find the custom node
# install(FILES bt_plugins.xml
#   DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
