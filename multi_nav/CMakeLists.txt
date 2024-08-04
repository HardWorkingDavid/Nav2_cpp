cmake_minimum_required(VERSION 3.8)
project(multi_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)  
find_package(nav_msgs REQUIRED)  
find_package(nav2_util REQUIRED)  
find_package(nav2_core REQUIRED)
find_package(tf2 REQUIRED)

find_package(angles REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(multi_nav src/multi_nav.cpp)
ament_target_dependencies(multi_nav rclcpp std_msgs geometry_msgs yaml-cpp tf2_geometry_msgs rcl_interfaces nav2_costmap_2d pluginlib nav_msgs nav2_util nav2_core tf2 angles rclcpp_action rclcpp_components nav2_msgs nav_2d_utils nav_2d_msgs)
target_link_libraries(multi_nav yaml-cpp::yaml-cpp)

install(TARGETS multi_nav DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
