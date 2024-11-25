cmake_minimum_required(VERSION 3.8)
project(robot_totalgrid_pkg_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  endif()
  add_executable(scan_node src/scan.cpp)
  ament_target_dependencies(scan_node rclcpp std_msgs geometry_msgs sensor_msgs)

  add_executable(tg_node src/totalgrid.cpp)
  ament_target_dependencies(tg_node rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2_geometry_msgs)
  
  install(TARGETS 
    tg_node
    scan_node
    DESTINATION lib/${PROJECT_NAME}
      )
  
  ament_package()
