cmake_minimum_required(VERSION 3.8)
project(nav_demo)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 11)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(nav_goal src/nav_goal.cpp)
ament_target_dependencies(nav_goal
  rclcpp
  geometry_msgs
  nav2_msgs
  rclcpp_action
)
install(TARGETS nav_goal
    DESTINATION lib/${PROJECT_NAME})
  



#install(DIRECTORY include DESTINATION share/${PROJECT_NAME})
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
