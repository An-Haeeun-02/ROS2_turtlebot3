# robot\_move\_pkg

`turtlebot3`가 키보드 입력에 따라 움직이는 프로그램입니다.

`w`, `a`, `d`, `x`, `s`를 입력하여 전직, 좌회전, 우회전, 환직, 정지 등의 움직음을 제어합니다.

---

## 패키지 설명

### 사용한 패키지

* **노드 생성을 위한 패키지**

  ```cpp
  #include "rclcpp/rclcpp.hpp"
  ```

* **메시지 타입 (이동 메시지)**

  ```cpp
  #include "geometry_msgs/msg/twist.hpp"
  ```

---

### 통신 방식

* **토픽 통신** : `/cmd_vel` 토픽으로 퍼블리시

---

## 코드 설명

### CMakeLists.txt

* **사용 패키지 추가**

  ```cmake
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  ```

* **노드 생성**

  ```cmake
  add_executable(move src/move.cpp)
  ament_target_dependencies(move rclcpp geometry_msgs)

  install(TARGETS
    move
    DESTINATION lib/${PROJECT_NAME})

  ament_package()
  ```

---

### package.xml

* **사용 패키지 등록**

  ```xml
  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  ```

---

### move.cpp

* `rodot_move` 라는 노드명을 갖는 프로그램
* 키보드 입력을 통해 `w`, `a`, `s`, `d`, `x` 로 로봇의 이동을 제어함
* `/cmd_vel` 토픽에 `Twist` 메시지를 발행하여 실제 이동을 구현함

---

### move2.cpp

* 로직은 동작하지만 복잡하여 사용하지 않음

---

## 작동 방법

```bash
colcon build --symlink-install --packages-select robot_move_pkg
. install/local_setup.bash
ros2 run robot_move_pkg move
```

텍스트 입력을 통해 프로그램을 실행하면, 입력된 문자에 따라 특정 동작을 수행합니다.

| 입력 | 작업            |
| -- | ------------- |
| w  | 전진 (forward)  |
| x  | 후진 (backward) |
| a  | 좌회전 (left)    |
| d  | 우회전 (right)   |
| s  | 정지 (stop)     |
