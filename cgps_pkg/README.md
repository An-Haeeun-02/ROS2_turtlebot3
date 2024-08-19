# cgps_pkg

gps 센서를 usb로 연결하여 제공하는 데이터 중 필요한 데이터만 추출하여 토픽으로 출력하

---

## 패키지 설명

### 사용한 패키지
- 노드를 위한 패키지

  ``#include "rclcpp/rclcpp.hpp" ``

- 메시지 타입을 위한 패키지
  ```
  #include "geometry_msgs/msg/vector3.hpp"
  ```
- gps 사용을 위한 외부 라이브러리

    ``#include <libgpsmm.h>``

    .libgpsmm 는 gpsd 패키지에 포함된 libgps 라이브러리

### 통신 방식
-  액션과 토픽을 통한 통신

    - ``/gps`` : 정제된 gps 값 전달을 위한 토픽


## 코드 설명
### CMakeList.txt

>사용한 패키지 추가
  ```
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
D)
  ```

> 외부 라이브러리 사용을 위한 설정
  ```
  # GPSD 라이브러리를 찾기 위한 pkg-config 사용
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPSD REQUIRED libgps)

  ```
> 노드 생성
   - gps_node

  액션 서버 노드. gps.cpp 실행
   - cli_node

액션 클라이언트 노드. cli.cpp 실행
  - scan_node

    정면, 좌측, 우측 거리 측정 스케너. scan.cpp 실행
  ```

add_executable(gps_node src/gps.cpp)
ament_target_dependencies(gps_node rclcpp sensor_msgs geometry_msgs)

# 타겟에 ROS2 의존성과 gpsd 라이브러리 링크
target_include_directories(gps_node PRIVATE ${GPSD_INCLUDE_DIRS})
target_link_libraries(gps_node ${GPSD_LIBRARIES})

install(TARGETS
  gps_node
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

### gps.cpp
  - /gps 토픽의 퍼블리셔

  - gps의 데이터 중 위도와 경도 데이터만 출력한다.


## 작동방법

### gps 사용법
```
//gpsd 서비스를 부팅 시 자동으로 시작
sudo systemctl enable gpsd.socket

//gpsd 소켓 서비스를 즉시 시작합니다.
sudo systemctl start gpsd.socket


//코드 확인
cgps -s

//상태확인
sudo systemctl status gpsd.socket

//코드 실행
g++ -o gps gps.cpp -lgps -lm

```
자세한 사용법은 아래의 링크를 참고해주세요
<https://wiki.52pi.com/index.php/EZ-0048>

### gps 노드(gps_node)
```
colcon build --symlink-install --packages-select cgps_pkg
. install/local_setup.bash
ros2 run cgps_pkg gps_node
```

