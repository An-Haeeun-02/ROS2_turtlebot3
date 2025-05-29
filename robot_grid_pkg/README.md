# robot\_grid\_pkg

`robot_grid_pkg`는 사용자가 입력한 2차원 격자 맵(Grid Map) 위에서 로봇의 경로를 자동으로 탐색하고, 이동 명령을 수행하는 ROS2 패키지입니다. A\* 기반 알고리즘으로 최적 경로를 탐색하고, 장애물을 회피하며 회전 지점도 함께 계산하여 실제 로봇을 이동시킬 수 있습니다.

---

## 📦 패키지 설명

### 사용한 패키지

* **ROS2 노드 작성**: `rclcpp`
* **로봇 제어 명령 메시지**: `geometry_msgs/msg/Twist`
* **로봇 위치 정보**: `nav_msgs/msg/Odometry`

### 실행 노드

* `grid_node`: `src/grid.cpp` 실행 파일. 전체 로직이 이 파일 하나에 포함되어 있음.

### 통신 구조

* **구독**: `odom` (로봇의 이동 거리 추적)
* **퍼블리시**: `cmd_vel` (로봇 이동/회전 명령)

---

## ⚙️ 동작 방식

### 1. 사용자 입력 기반 그리드 생성

* 너비, 높이, 시작점, 목표점, 장애물 수 및 각각의 위치와 크기를 터미널에서 입력합니다.
* 장애물 위치는 그리드 상에 표시됩니다.

### 2. A\* 알고리즘 기반 경로 탐색

* 시작점부터 목표점까지 장애물을 피해서 갈 수 있는 최적 경로를 계산합니다.
* 방향 전환이 필요한 지점을 회전 지점으로 판단하여 저장합니다.

### 3. 경로 시각화 출력

* ASCII 문자를 이용하여 콘솔에 경로를 시각적으로 출력합니다.

  * `*`: 시작점과 목표점
  * `+`: 회전 지점
  * `#`: 장애물
  * 공백: 이동 가능 경로

### 4. 로봇 실제 이동 수행

* 경로 정보를 기반으로

  * 전진 (`go_straight`)
  * 좌/우 회전 후 전진 (`left_course`, `right_course`)
    을 수행합니다.

* `odom` 데이터를 바탕으로 로봇이 얼마나 이동했는지를 확인하며 정확하게 거리를 측정해 멈춥니다.

---

## 🧾 코드 구성 요소 설명

### 주요 클래스 및 함수

* **TurtlebotController**: 메인 ROS2 노드 클래스

  * `make_root()`: 사용자로부터 입력 받아 그리드 생성 및 경로 탐색
  * `find_path()`: A\* 기반 경로 탐색 알고리즘
  * `print_grid_with_path()`: 경로 시각화 출력
  * `move_robot()`: 경로대로 실제 로봇 이동
  * `odom_callback()`: 이동 거리 계산을 위한 오도메트리 처리

### 주행 명령 함수

* `go_straight(int block, double cm)`: 지정된 칸 수만큼 전진
* `left_course(int block, double cm)`: 좌회전 후 전진, 다시 우회전
* `right_course(int block, double cm)`: 우회전 후 전진, 다시 좌회전

---

## 🛠️ 설정 파일

### CMakeLists.txt

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(grid_node src/grid.cpp)
ament_target_dependencies(grid_node rclcpp geometry_msgs nav_msgs)

install(TARGETS grid_node DESTINATION lib/${PROJECT_NAME})
ament_package()
```

### package.xml

```xml
<package format="3">
  <name>robot_grid_pkg</name>
  <version>0.0.0</version>
  <maintainer email="godma531@naver.com">hae</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 🏁 실행 방법

```bash
ros2 run robot_grid_pkg grid_node
```

실행 후 터미널에서 다음 정보를 입력:

1. 격자 크기 (너비/높이)
2. 시작점 좌표
3. 도착점 좌표
4. 장애물 개수, 위치(x, y), 크기(너비, 높이)

---

## ✅ 예시 출력

```
@ @ @ @ @ @ @
@ *       * @
@   # # #   @
@   +   +   @
@ @ @ @ @ @ @
```

* `*`: 시작점/목표점
* `#`: 장애물
* `+`: 회전 지점

---

## 📌 특징

* 사용자 입력 기반 동적 맵 구성 가능
* 회전 지점 자동 탐지 및 제어 가능
* 오도메트리를 활용한 정밀한 거리 제어
* 콘솔 기반 시각화 지원
