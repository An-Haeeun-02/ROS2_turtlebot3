# robot_cleaner_pkg

`turtlebot3`가 장애물을 감지하여 자동으로 회피합니다.

정면과 장애물의 거리가 30보다 적을 시 정지하고, 좌우의 장애물 거리를 감지하여 장애물과 더 먼 쪽으로 회전해 이동합니다.

---

## 패키지 설명

### 사용한 패키지

* **노드 생성을 위한 패키지**

  ```cpp
  #include "rclcpp/rclcpp.hpp"
  ```

* **메시지 타입 패키지**

  ```cpp
  // 퍼블리시어 메시지 타입 (ubcf4내는  \uc쵸)
  #include "geometry_msgs/msg/twist.hpp"

  // 서브스크라이버 메시지 타입 (ubc1b는  \uc쵸)
  #include "geometry_msgs/msg/vector3.hpp"
  ```

---

### 통신 방식

* **토픽 거래**

  * `/cmd_vel`: Turtlebot 제어 토픽
  * `/scan`: Turtlebot 삼성 센서 토픽
  * `/aeeun`: 정제된 센서 정보 토픽

---

## 코드 설명

### CMakeLists.txt

* **사용 패키지 추가**

  ```cmake
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  ```

* **노드 생성**

  ```cmake
  add_executable(cleaner src/cleaner.cpp)
  ament_target_dependencies(cleaner rclcpp std_msgs geometry_msgs)

  add_executable(scan_node src/scan.cpp)
  ament_target_dependencies(scan_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)

  install(TARGETS
    cleaner
    scan_node
    DESTINATION lib/${PROJECT_NAME})

  ament_package()
  ```

---

### package.xml

* **의존 패키지 등록**

  ```xml
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <de법
### 센서 노드 (scan\_node)

```bash
colcon build --symlink-install --packages-select robot_cleaner_pkg
. install/local_setup.bash
ros2 run robot_cleaner_pkg scan_node
```

### 동작 노드 (cleaner)

```bash
colcon build --symlink-install --packages-select robot_cleaner_pkg
. install/local_setup.bash
ros2 run robot_cleaner_pkg cleaner
```
