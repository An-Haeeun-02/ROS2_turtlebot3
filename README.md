# ROS2기반으로 turtlebot3를 사용한 로봇 프로그래밍

이 프로젝트는 ~~

## 🔧 기술 스택
- **운영체제**: Ubuntu 20.04
- **로봇 플랫폼**: TurtleBot3 (Burger)
- **프레임워크**: ROS 2 <img src="https://img.shields.io/badge/ros-22314E?style=for-the-badge&logo=ros&logoColor=white"> (Humble)
- **언어**: <img src="https://img.shields.io/badge/c++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white">
- **빌드 툴**: colcon
  

## 📁 프로젝트 디렉토리 구조
- `.vscode/` : VSCode 설정
- `Cgps/` : GPS 센서 관련 코드
- `action_test/` : ROS2 Action 테스트용 코드
- `build/` : colcon 빌드 결과물 저장
- `cgps_pkg/` : GPS 수신 기능 패키지
- `dot_pkg/` : 도트 기반 시각화 또는 경로 표시 기능
- `install/` : 빌드 후 설치 파일
- `log/` : 실행 로그 저장
- `robot_action/` : ROS2 action 기능 정의
- `robot_cleaner_pkg/` : 장애물 회피 로직
- `robot_drive_pkg/` : 로봇 기본 주행 기능
- `robot_go_pkg/` : 목표 지점까지 이동 로직
- `robot_grid_pkg/` : 그리드 기반 경로 탐색 알고리즘
- `robot_move_pkg/` : 키보드 입력으로 로봇 제어 (Teleop)
- `robot_total_pkg/` : 전체 기능 통합 패키지
- `robot_totalgrid_pkg/` : 좌표 기반 장애물 회피 + 전체 통합
- `robot_totalgrid_pkg_v2/` : 개선된 통합 그리드 버전
- `test_pkg/` : 액션 서버/클라이언트 실험용 코드
- `README.md` : 프로젝트 설명 파일



## 📋주요 기능 및 패키지
1. ### robot_move_pkg
  -  토픽 통신 방식을 통해 키보드 입력에 따라 움직임
  
2. ### rogot_cleaner_pkg
  -  직진 움직임 중 토픽 통신 방식으로 정면의 장애물을 감지하고 회피하여 움직임

3. ### robot_drive_pkg
  - 직진 움직임 중 토픽 통신 방식으로 정면과 좌우의 장애물을 감지하고 더 먼 쪽으로 회피하여 원래 경로로 복귀

4. ### robot_go_pkg
- 액션 통신 방식으로 장애물을 감지하고 입력된 거리만큼 움직이면서 장애물을 피함

5. ### robot_grid_pkg
  - 그리드 범위 내에서 시작점과 도착점의 경로를 계산하여 A 알고리즘을 바탕으로 최적화 된 경로대로 움직임
    
6. ### robot_total_pkg
   - IMU를 사용하여 방향 정확도 개선 및 계단 등의 장애물을 감지하고 회피함.
     
7. ### robot_totalgrid_pkg
    - 사용자로부터 맵정보, 시작과 목표지점, 장애물 위치를 입력받음
    - A*알고리즘을 사용해 최적 경로를 계싼하고 회전 지점을 추출함.
    - 경로를 따라 주행하며 기존의 장애물을 피하거나, 새로운장애물을 인식하여 맵에 반영하고 경로를 재탐색하여 목적지에 도착
8. ### robot_totalgrid_pkg_v2
    - robot_totalgrid_pkg에서 가독성과 유지보수성을 보완함
    
9. ### robot_action
- 액션 형식이 담겨 있는 패키지
- 
10. ### cgps_pkg
   - EZ-0048 GPS 센서 드라이버 연동히여 위도·경도 정보를 터미널에 출력

11. ### dot_pkg
   - 일정 반경의 장애물을 도트 형식으로 시각화

## 🚀 설치 및 실행 방법
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

## 📄 논문
이 프로젝트 진행중 작성한 논문.
<<휴머노이드 로봇 제어를 위한 ROS 노드 설계>>
https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE11860907

##🏆 공모전(캡스톤디자인)
지자체-대학 협력기반 지역혁신 사업 프로젝트랩 캡스톤디자인 - 의료기기를 위한 ROS2 포팅 프로젝트 랩

##📜 라이선스
MIT License

##👩‍💻 작성자
안해은 외 4명
  
