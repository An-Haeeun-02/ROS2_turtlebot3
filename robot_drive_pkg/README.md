# robot\_drive\_pkg

`turtlebot3`가 정면, 우측, 좌측 범위의 장애물과 거리를 감지하여 자동적으로 움직이고 회피하는 프로그램입니다.

일정 거리 사이에 장애물이 없으면 통과하고, 가장 거리가 먼 방향으로 회피 동작을 수행합니다.
벽으로 인식되는 물체를 만나면 정지합니다.

---

## 패키지 설명

### 사용한 패키지

* **노드 생성을 위한 패키지**

  ```cpp
  #include "rclcpp/rclcpp.hpp"
  ```

* **메시지 타입**

  ```cpp
  #include "geometry_msgs/msg/twist.hpp"
  #include "geometry_msgs/msg/vector3.hpp"
  #include <sensor_msgs/msg/laser_scan.hpp>
  ```

* **기본 버튼 / 시간 / 소스 관련**

  ```cpp
  #include <chrono>
  #include <thread>
  #include <csignal>
  #include <map>
  ```

---

### 통신 방식

* **토픽 거래**

  * `/cmd_vel`: turtlebot 제어 토픽
  * `/scan`: 센서 데이터 토픽
  * `/aeeun`: 정면 거리 토픽
  * `/hyewon`: 우측 거리 토픽
  * `/dongwan`: 좌측 거리 토픽

---

## 코드 설명

### CMakeLists.txt

* **사용 패키지 추가**

  ```cmake
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  ```

* **노드 생성**

  ```cmake
  add_executable(scan_node src/scan.cpp)
  ament_target_dependencies(scan_node rclcpp std_msgs geometry_msgs sensor_msgs)

  add_executable(drive_node src/drive.cpp)
  ament_target_dependencies(drive_node rclcpp std_msgs geometry_msgs)

  install(TARGETS
    drive_node
    scan_node
    DESTINATION lib/${PROJECT_NAME})

  ament_package()
  ```

---

### package.xml

* **사용 패키지 등록**

  ```xml
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  ```

---

### drive.cpp

* `/cmd_vel` 퍼블리셔
* `/aeeun`, `/hyewon`, `/dongwan` 서브스크라이버 (정면/우/좌 센서)
* 정해진 거리 이하로 0도, 30도, 60도, 300도, 330도의 값이 떨어지면 종료(벽으로 인식)
* 장애물 감지 시 좌/우 센서 비교하여 회피 동작 수행
* 센서 기반 복합 경로 회피 로직 포함 (right\_course, left\_course)
* signalHandler를 통해 Ctrl+C 종료 시 멈춤 명령 발행

---

### scan.cpp

* `/scan` 서브스크라이버
* `/aeeun`, `/hyewon`, `/dongwan` 퍼블리셔
* 0°, 30°, 60°, 90°, 270°, 300°, 330° 방향의 거리 데이터를 Vector3 메시지로 전송
* 센서 측정값을 cm 단위로 변환 후 사용

---

## 작동 방법

### 센서 노드 실행 (scan\_node)

```bash
colcon build --symlink-install --packages-select robot_drive_pkg
. install/local_setup.bash
ros2 run robot_drive_pkg scan_node
```

### 주행 제어 노드 실행 (drive\_node)

```bash
colcon build --symlink-install --packages-select robot_drive_pkg
. install/local_setup.bash
ros2 run robot_drive_pkg drive_node
```
