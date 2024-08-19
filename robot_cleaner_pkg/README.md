# robot_cleaner_pkg

turtlebot3가 장애물을 감지하여 스스로 회피한다.

정면과 장애물의 거리가 30보다 적을 시 정지하며, 좌우의 장애물 거리를 감지하여 장애물과 더 먼쪽으로 회전하여 이동한다.

---

## 패키 설명

### 사용한 패키지
- 노드를 위한 패키지

  ``#include "rclcpp/rclcpp.hpp" ``

- 메시지 타입 패키지

  ```
  //퍼블리셔 메시지 타입(보내는 쪽) 
  #include "geometry_msgs/msg/twist.hpp"

  //서브스크라이버 메시지 타입(받는 쪽)
  #include "geometry_msgs/msg/vector3.hpp" 
  ```

### 통신 방식
-  토픽을 통한 통신

    - ``/cmd_vel`` : tutlebot 제어 토픽
    - ``/scan`` : tutlebot 센서 토픽
    - ``/aeeun`` : 정제된 센서 정보 토

## 코드 설명
### CMakeList.txt

>사용한 패키지 추가
  ```
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  ```
> 노드 생성
  - cleaner

    장애물 회피와 움직임 담당
  - scan_node

    정면, 좌측, 우측 거리 측정 스케너
  ```
  add_executable(cleaner src/cleaner.cpp)
  ament_target_dependencies(cleaner rclcpp  std_msgs geometry_msgs)
  add_executable(scan_node src/scan.cpp)
  ament_target_dependencies(scan_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)

  install(TARGETS 
    cleaner
    scan_node
      DESTINATION lib/${PROJECT_NAME})
  ament_package()
  ```
### pakage.xml
> 사용한 패키지 접근성 추가
  ```
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  ```

### cleaner.cpp
  - /cmd_vel 토픽의 퍼블리셔

  - /aeeun 토픽의 서브스크라이버
  
  - 센서 데이터를 받을 때 마다 콜백함수 호출을 통해 장애물과의 거리를 검사하고, 회피로직에 따른 적절한 동작 수행.

### scan.cpp
  - /scan 토픽의 서브스크라이버

  - /aeeun 토픽의 퍼블리셔

  - 토픽을 통해 전달되는 센서 데이터 중 정면, 우측, 좌측의 값만 vector3 형태의 메시지에 (정면,우측, 좌측) 값을 넣어 퍼블리시함.

---

## 작동방법

### 센서 노드(scan_node)
```
colcon build --symlink-install --packages-select robot_cleaner_pkg
. install/local_setup.bash
ros2 run robot_cleaner_pkg scan_node
```

### 동작 노드(cleaner)
```
colcon build --symlink-install --packages-select robot_cleaner_pkg
. install/local_setup.bash
ros2 run robot_cleaner_pkg cleaner
```
