cmake_minimum_required(VERSION 3.8)
project(cgps_pkg)
# ROS2 패키지 의존성 설정
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# GPSD 라이브러리를 찾기 위한 pkg-config 사용
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPSD REQUIRED libgps)

# C++14 표준 사용
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 실행 파일 생성
add_executable(gps_node src/gps.cpp)

# 타겟에 ROS2 의존성과 gpsd 라이브러리 링크
ament_target_dependencies(gps_node rclcpp sensor_msgs geometry_msgs)
target_include_directories(gps_node PRIVATE ${GPSD_INCLUDE_DIRS})
target_link_libraries(gps_node ${GPSD_LIBRARIES})

# 설치 설정
install(TARGETS
  gps_node
  DESTINATION lib/${PROJECT_NAME}
)

# 패키지 선언
ament_package()