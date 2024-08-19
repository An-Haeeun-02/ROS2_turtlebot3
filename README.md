# ROS2기반으로 turtlebot3를 사용한 로봇 프로그래밍

  <img src="https://img.shields.io/badge/c++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white"> <img src="https://img.shields.io/badge/ros-22314E?style=for-the-badge&logo=ros&logoColor=white">

  쉬운 동작부터 관련된 다양한 통신을 통한 방법까지 익혀본다.
---

## 패키지 생성/빌드
### 패키지 생성
```
cd robot_ws/src
ros2 pkg create 패키지명 --build-type ament_cmake --dependencies 필요 패키지
//자주 쓰는 패키지
ros2 pkg create 패키지명 --build-type ament_cmake --dependencies rclcpp std_msgs
```

### 패키지 빌드

````
colcon build --symlink-install --packages-select 패키지명
. install/local_setup.bash
ros2 run 패키지명 지정노드명
````

### 로봇 빌드
````
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
````

---

## 패키지 설명
1. ### robot_move_pkg
-  토픽 통신 방식을 통해 키보드 입력을 통해 움직이는 패키지
  
2. ### rogot_cleaner_pkg
-  토픽 통신 방식으로 장애물을 감지하고 움직이는 패키지

3. ### action_test
-  액션 동작을 확인할 수 있는 간단한 액션 패키지

4. ### robot_action
- 액션 형식이 담겨 있는 패키지
  
5. ### robot_go_pkg
- 액션 통신 방식으로 장애물을 감지하고 입력된 거리만큼 움직이면서 장애물을 피해가는 패키

6. ### cgps_pkg
- EZ-0048 GPS 센서를 통해 좌표값만 출력하는 패키

7.  ### test_pkg
-  패키지를 만들기 전, 테스트 용으로 만드는 패키지
  
