# robot_move_pkg

turtlebot3의 움직임을 키보드를 통해 제어하는 프로그램.

w,a,d,x,s 를 통해 움직임을 제어한다.

---

## 패키 설명

### 사용한 패키지
- 노드를 위한 패키지

  ``#include "rclcpp/rclcpp.hpp" ``

- 메시지 타입 패키지

  ``#include "geometry_msgs/msg/twist.hpp"``

### 통신 방식
-  토픽 통신 : ``cmd_vel``

## 코드 설명
### CMakeList.txt
- 사용한 패키지 추가

  ``find_package(rclcpp REQUIRED)``
  
  ``find_package(geometry_msgs REQUIRED)``
- 노드 생성

- move 라는 이름의 노드에 move.cpp 파일 지정
  ```
  add_executable(move src/move.cpp)
  ament_target_dependencies(move rclcpp geometry_msgs)
  install(TARGETS 
    move 
      DESTINATION lib/${PROJECT_NAME})
  ament_package()
  ```
### pakage.xml
- 사용한 패키지 접근성 추가

    ``<depend>rclcpp</depend>``
  
    ``<depend>geometry_msgs</depend>``
### move.cpp
- 로봇을 움직이는 main 프로그램. 키보드 동작을 통해 움직임.

- ``rodot_move``라는 노드명 부여

### move2.cpp
- 로봇을 움직이는 프로그램. 정상 동작하나 로직이 어려워 사용하지 않음

---

## 작동방법

```
colcon build --symlink-install --packages-select robot_move_pkg
. install/local_setup.bash
ros2 run robot_move_pkg move
```
