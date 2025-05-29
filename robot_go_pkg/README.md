# robot\_go\_pkg

`turtlebot3`가 장애물을 감지하여 입력된 거리만큼 이동하며 자동으로 회피합니다.

액션을 통해 움직일 거리를 입력 받고, 이동한 거리만큰 피드백을 통해 서버에 알립니다.

정면과 장애물과의 거리가 30보다 적을 경우 정지하고, 좌우 거리를 감지하여 장애물과 더 먼 쪽으로 회전합니다.

---

## 패키지 설명

### 사용한 패키지

* **노드 생성을 위한 패키지**

  ```cpp
  #include "rclcpp/rclcpp.hpp"
  ```

* **액션 노드 생성을 위한 패키지**

  ```cpp
  #include <rclcpp_action/rclcpp_action.hpp>
  ```

* **사용자 지정 액션 메시지 타입**

  ```cpp
  #include "robot_action/action/move.hpp"
  ```

* **메시지 타입을 위한 패키지**

  ```cpp
  #include <nav_msgs/msg/odometry.hpp>
  #include <geometry_msgs/msg/twist.hpp>
  #include <geometry_msgs/msg/vector3.hpp>
  #include <geometry_msgs/msg/point.hpp>
  ```

---

### 통신 방식

* **액션과 토픽 통신**

  * `/cmd_vel`: turtlebot3 제어 토픽
  * `/scan`: 센서 거래 토픽
  * `/odom`: 거리 측정 토픽
  * `/aeeun`: 정제된 센서 정보 토픽
  * `/an`: 액션 서버 이름

---

## 코드 설명

### CMakeLists.txt

* **사용한 패키지 추가**

  ```cmake
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(action_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(robot_action REQUIRED)
  find_package(rclcpp_action REQUIRED)
  ```

* **노드 생성**

  ```cmake
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
    DESTINATION lib/${PROJECT_NAME})

  ament_package()
  ```

---

### package.xml

* **의존 패키지 등록**

  ```xml
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>rclcpp_action</depend>
  <depend>robot_action</depend>
  ```

---

### cli.cpp

* `/an` 액션의 클라이언트
* 서버와 연결되면 이동거리를 입력 받음
* 로봇 이동거리 피드백 수신 및 완료 메시지 수신

---

### go.cpp

* `/cmd_vel` 퍼블리셔
* `/aeeun`, `/odom` 서브스크라이버
* `/an` 액션 서버
* 이동할 목표 거리 입력 받아 장애물 회피하며 이동
* 이동 거리 피드백을 클라이언트에 전달

---

### scan.cpp

* `/scan` 서브스크라이버
* `/aeeun` 퍼블리셔
* 정면, 좌측, 우측 거리 정보를 Vector3 메시지로 퍼블리시

---

## 작동 방법

### 사용자 지정 액션 메시지 빌드

```bash
colcon build --symlink-install --packages-select robot_action
. install/local_setup.bash
```

### 센서 노드 실행 (scan\_node)

```bash
colcon build --symlink-install --packages-select robot_go_pkg
. install/local_setup.bash
ros2 run robot_go_pkg scan_node
```

### 서버 노드 실행 (go\_node)

```bash
colcon build --symlink-install --packages-select robot_go_pkg
. install/local_setup.bash
ros2 run robot_go_pkg go_node
```

### 클라이언트 노드 실행 (cli\_node)

```bash
colcon build --symlink-install --packages-select robot_go_pkg
. install/local_setup.bash
ros2 run robot_go_pkg cli_node
```
