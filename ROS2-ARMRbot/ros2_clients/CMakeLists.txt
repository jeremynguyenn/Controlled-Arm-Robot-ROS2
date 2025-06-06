cmake_minimum_required(VERSION 3.8)
project(ros2_clients)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_actions REQUIRED)
find_package(ros2_data REQUIRED)
find_package(ros2_grasping REQUIRED)
find_package(tu_int REQUIRED)
find_package(linkattacher_msgs REQUIRED)

add_executable(actor src/gazebo_actor.cpp)
ament_target_dependencies(actor rclcpp rclcpp_action ros2_data tu_int ros2_actions ros2_grasping std_msgs linkattacher_msgs)

include_directories(include)
install(DIRECTORY include/
DESTINATION include
)

install(TARGETS
  actor
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
