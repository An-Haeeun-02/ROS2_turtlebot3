# cgps\_pkg

`cgps_pkg`·는 USB·로 연결된 GPS 센서에서 필요한 데이터(위도, 경도)만 추출하여 ROS 2 토픽으로 출력하는 패키지입니다.

---

## 🏦 패키지 설명

### 사용한 패키지

* **노드 생성을 위한 패키지**

  ```cpp
  #include "rclcpp/rclcpp.hpp"
  ```

* **메시지 타입 사용을 위한 패키지**

  ```cpp
  #include "geometry_msgs/msg/vector3.hpp"
  ```

* **GPS 사용을 위한 외부 라이브러리**

  ```cpp
  #include <libgpsmm.h>
  ```

  > `libgpsmm`·은 `gpsd` 패키지에 포함된 `libgps` 라이브러리입니다.

---

### 통신 방식

* **통신을 이용한 토픽**

  * `/gps`: 정제된 GPS 위도 및 경도 데이터를 퍼블리시한다.

---

## 🧾 코드 설명

### CMakeLists.txt

* **사용 패키지 추가**

  ```cmake
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  ```

* **외부 라이브러리 설정**

  ```cmake
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GPSD REQUIRED libgps)
  ```

* **노드 설정**

  ```cmake
  add_executable(gps_node src/gps.cpp)
  ament_target_dependencies(gps_node rclcpp sensor_msgs geometry_msgs)

  target_include_directories(gps_node PRIVATE ${GPSD_INCLUDE_DIRS})
  target_link_libraries(gps_node ${GPSD_LIBRARIES})

  install(TARGETS
    gps_node
    DESTINATION lib/${PROJECT_NAME}
  )
  ament_package()
  ```

---

### package.xml

* **의존 패키지**

  ```xml
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>rclcpp_action</depend>
  <depend>robot_action</depend>
  ```

---

### gps.cpp

* `/gps` 토픽 퍼블리시어 노드입니다.
* GPS의 위도(`latitude`)와 경도(`longitude`) 값만 추출하여 퍼블리시합니다.
* 높이(`altitude`)는 사용하지 않으며 `0.0`으로 고정합니다.

---

## 🚀 작동 방법

### 1. GPS 서비스 시작

```bash
# 부팅 시 gpsd 자동 시행
sudo systemctl enable gpsd.socket

# 즉시 gpsd 시행
sudo systemctl start gpsd.socket

# GPS 연결 확인
cgps -s

# 서비스 상태 확인
sudo systemctl status gpsd.socket
```

> 상세 설정은 다음 링크를 참고하세요:
> 🔗 [https://wiki.52pi.com/index.php/EZ-0048](https://wiki.52pi.com/index.php/EZ-0048)

---

### 2. 코드 실행 (단독 실행 시)

```bash
g++ -o gps gps.cpp -lgps -lm
./gps
```

---

### 3. ROS 2 노드 실행

```bash
# 빌드
colcon build --symlink-install --packages-select cgps_pkg

# 환경 설정
. install/local_setup.bash

# gps_node 실행
ros2 run cgps_pkg gps_node
```
