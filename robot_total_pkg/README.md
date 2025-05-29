# robot\_total\_pkg

`robot_total_pkg`는 지금까지 제작한 패키지를 한번에 제공합니다.
주행 시 장애물 회피, 계단 감지, 방향 유지 등의 기능을 통합 제공합니다. 또한 LaserScan 센서를 기반으로 센서 데이터를 Vector3 메시지로 변환해 퍼블리시하는 기능도 포함됩니다.
IMU를 사용하여 방향 정확도 개선 및 계단 등의 장애물을 감지하고 회피합니다.
## 주요 구성 노드

### 1. `drive_node`

로봇 주행을 담당하며, 센서 정보를 기반으로 실시간 장애물 회피 및 방향 보정을 수행합니다.

* 센서 주행 기반 자동 제어
* IMU 센서로 방향 유지 및 보정
* 장애물 회피 시 회전 및 후진 포함
* 계단 감지 시 정지

**주요 동작 흐름:**

1. LaserScan 데이터를 수신하여 전방/좌우 거리 확인
2. 장애물이 감지되면 방향에 따라 회전 수행
3. 각도 차이 발생 시 IMU 기반 각도 보정
4. 모든 센서값이 일정 임계값 이하가 되면 주행 종료

### 2. `scan_node`

LaserScan 센서 데이터를 Vector3 타입 메시지로 변환하여 퍼블리시하는 역할을 수행합니다.

* `/scan` 토픽으로부터 LaserScan 수신
* 0, 30, 60, 90, 270, 300, 330도 방향 거리 측정
* 유효한 데이터만 필터링하여 전용 토픽(`aeeun`, `hyewon`, `dongwan`)으로 퍼블리시

## 토픽 구조

| 토픽 이름      | 메시지 타입                      | 설명               |
| ---------- | --------------------------- | ---------------- |
| `/scan`    | `sensor_msgs/msg/LaserScan` | Lidar 스캔 데이터 수신  |
| `/cmd_vel` | `geometry_msgs/msg/Twist`   | 로봇 이동 명령         |
| `/imu`     | `sensor_msgs/msg/Imu`       | 방향 추정을 위한 IMU 정보 |
| `aeeun`    | `geometry_msgs/msg/Vector3` | 정면, 90도, 270도 거리 |
| `hyewon`   | `geometry_msgs/msg/Vector3` | 30도, 60도 거리      |
| `dongwan`  | `geometry_msgs/msg/Vector3` | 300도, 330도 거리    |

## 실행 방법

```bash
# 빌드
colcon build --packages-select robot_total_pkg

# 실행 (터미널 1)
ros2 run robot_total_pkg scan_node

# 실행 (터미널 2)
ros2 run robot_total_pkg drive_node
```

## 동작 예시

1. LaserScan → Vector3 변환 후 퍼블리시
2. drive\_node는 센서값을 수신하여 장애물 탐지
3. 회피 동작 수행 (우회전, 좌회전, 정지)
4. IMU 기반 방향 보정
5. 모든 방향에서 장애물이 없으면 자동 정지

## 의존성

* `rclcpp`
* `std_msgs`
* `geometry_msgs`
* `sensor_msgs`
* `nav_msgs`
* `tf2_geometry_msgs`

## 빌드 및 설치

`CMakeLists.txt`와 `package.xml` 설정은 ROS2 Foxy/ Humble 기반에서 작동하도록 구성되어 있습니다.

```bash
colcon build --packages-select robot_total_pkg
source install/setup.bash
```

## 참고 사항

* `/cmd_vel` 퍼블리시는 `geometry_msgs/msg/Twist` 기준으로 속도 및 회전 명령을 전달합니다.
* LaserScan에서 NaN이나 0 값은 무시됩니다.
* 각 센서 방향별 임계값은 코드 내 `sensor_thresholds_`에서 설정 가능하며 조정 가능합니다.

---

이 패키지는 복잡한 주행 환경에서도 로봇이 자율적으로 장애물을 피하며 안전하게 주행하도록 설계되었습니다.
