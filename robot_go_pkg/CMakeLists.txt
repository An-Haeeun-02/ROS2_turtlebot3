cmake_minimum_required(VERSION 3.8)
project(robot_go_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_action REQUIRED)
find_package(rclcpp_action REQUIRED)

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

#add_executable(test_node src/test.cpp)
#ament_target_dependencies(test_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)
#add_executable(scan_node src/scan.cpp)
#ament_target_dependencies(scan_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)
#add_executable(go_node src/go.cpp)
#ament_target_dependencies(go_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)
add_executable(server_node src/actionsever.cpp)
ament_target_dependencies(server_node rclcpp rclcpp_action robot_action)
add_executable(cli_node src/actioncli.cpp)
ament_target_dependencies(cli_node rclcpp rclcpp_action robot_action)

install(TARGETS 
  #test_node
  #scan_node
  #go_node
  server_node
  cli_node
  DESTINATION lib/${PROJECT_NAME}
    )
ament_package()
