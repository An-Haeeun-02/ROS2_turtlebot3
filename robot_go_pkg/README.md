# robot_go_pkg

turtlebot3가 장애물을 감지하여 입력 된 거리만큼 움직이며 스스로 회피한다.

액션을 통해 움직일 거리를 입력 받고, 이동한 거리만큼 피드백을 보낸다.

정면과 장애물의 거리가 30보다 적을 시 정지하며, 좌우의 장애물 거리를 감지하여 장애물과 더 먼쪽으로 회전하여 이동한다.

---

## 패키지 설명

### 사용한 패키지
- 노드를 위한 패키지

  ``#include "rclcpp/rclcpp.hpp" ``

- 액션 노드 제작 패키지

  ``#include <rclcpp_action/rclcpp_action.hpp>``

- 사용자 지정 액션메시지 타입 사용

  ``#include "robot_action/action/move.hpp" ``

    robot_action 패키지에 move.hpp라는 액션 메시지 형태 제작
- 메시지 타입을 위한 패키지
  ```
  #include <nav_msgs/msg/odometry.hpp> 
  #include <geometry_msgs/msg/twist.hpp> 
  #include <geometry_msgs/msg/vector3.hpp>
  #include <geometry_msgs/msg/point.hpp>
  ```

### 통신 방식
-  액션과 토픽을 통한 통신

    - ``/cmd_vel`` : tutlebot 제어 토픽
    - ``/scan`` : tutlebot 센서 토픽
    - ``/odom`` : tutlebot 거리 측정 토픽
    - ``/aeeun`` : 정제된 센서 정보 토픽
    - ``/an`` : 생성한 액션의 이름

## 코드 설명
### CMakeList.txt

>사용한 패키지 추가
  ```
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(action_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(robot_action REQUIRED)
  find_package(rclcpp_action REQUIRED)
D)
  ```
> 노드 생성
   - go_node

  액션 서버 노드. go.cpp 실행
   - cli_node

액션 클라이언트 노드. cli.cpp 실행
  - scan_node

    정면, 좌측, 우측 거리 측정 스케너. scan.cpp 실행
  ```
  add_executable(scan_node src/scan.cpp)
  ament_target_dependencies(scan_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)

  add_executable(go_node src/go.cpp)
  ament_target_dependencies(go_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs rclcpp_action robot_action)

  add_executable(cli_node src/cli.cpp)
  ament_target_dependencies(cli_node rclcpp rclcpp_action robot_action)

  install(TARGETS 
    scan_node
    go_node
    cli_node
    DESTINATION lib/${PROJECT_NAME}
      )
  ament_package()
  ```
### pakage.xml
> 사용한 패키지 접근성 추가
  ```
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>rclcpp_action</depend>
  <depend>robot_action</depend>
  ```

### cli.cpp
  - /an 액션의 클라이언트.
  
  - 서버와 연결되면 이동거리를 입력 받는다.
  - 로봇의 이동거리를 피드백 받고, 이동이 완료되면 완료 메시지를 받는다.

### go.cpp
  - /cmd_vel 토픽의 퍼블리셔

  - /aeeun 토픽의 서브스크라이버

  - /odom 토픽 서브스크라이버

  - /an 액션의 서버
  
  - 클라이온트로부터 이동거리를 입력받는다.
 - 원하는 거리만큼 장애물을 피해가며 이동한다.
 - 이동거리를 클라이언트로 피드백 보낸다.

### scan.cpp
  - /scan 토픽의 서브스크라이버

  - /aeeun 토픽의 퍼블리셔

  - 토픽을 통해 전달되는 센서 데이터 중 정면, 우측, 좌측의 값만 vector3 형태의 메시지에 (정면,우측, 좌측) 값을 넣어 퍼블리시함.

---

## 작동방법

### 사용자 지정 액션 메시지 빌드
```
colcon build --symlink-install --packages-select robot_action
. install/local_setup.bash
```

### 센서 노드(scan_node)
```
colcon build --symlink-install --packages-select robot_go_pkg
. install/local_setup.bash
ros2 run robot_go_pkg scan_node
```

### 서버 동작 노드(go_node)
```
colcon build --symlink-install --packages-select robot_go_pkg
. install/local_setup.bash
ros2 run robot_go_pkg go_node
```

### 클라이언트 동작 노드(cli_node)
```
colcon build --symlink-install --packages-select robot_go_pkg
. install/local_setup.bash
ros2 run robot_go_pkg cli_node
```
