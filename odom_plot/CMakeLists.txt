cmake_minimum_required(VERSION 3.8)
project(odom_plot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(odom_write src/odom_write.cpp)
ament_target_dependencies(odom_write rclcpp std_msgs nav_msgs)

add_executable(odom_load src/odom_load.cpp)
ament_target_dependencies(odom_load rclcpp std_msgs geometry_msgs visualization_msgs)

add_executable(plan_write src/plan_write.cpp)
ament_target_dependencies(plan_write rclcpp std_msgs nav_msgs)

add_executable(plan_load src/plan_load.cpp)
ament_target_dependencies(plan_load rclcpp std_msgs geometry_msgs visualization_msgs)

install(TARGETS
  odom_write
  odom_load
  plan_write
  plan_load
  DESTINATION lib/${PROJECT_NAME}
)

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
