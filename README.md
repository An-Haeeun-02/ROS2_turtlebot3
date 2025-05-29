# ROS2기반 turtlebot3 프로그래밍
<img src="https://img.shields.io/badge/c++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white"><img src="https://img.shields.io/badge/ros-22314E?style=for-the-badge&logo=ros&logoColor=white">

이 프로젝트는 ROS2 기반으로 TurtleBot3의 움직임을 제어하기 위한 패키지 모음입니다.
기본적인 이동부터 로봇의 다양한 이동방법과 장애물 감지 및 회피 로직을 담고 있습니다.
TurtleBot3내의 센서및 ROS2의 기능을 활용하여 GPS통신과 새로운 센서 추가 등의 기능을 실습하고 개발하였습니다.

## 🔧 기술 스택
- **운영체제**: Ubuntu 20.04
- **로봇 플랫폼**: TurtleBot3 (Burger)
- **프레임워크**: ROS 2 (Humble)
- **언어**: C++

---  

## 📁 프로젝트 디렉토리 구조
```
ROS2_turtlebot3/
├── .vscode/ # VSCode 설정
├── Cgps/ # GPS 센서 관련 코드
├── action_test/ # ROS2 Action 테스트용
├── build/ # colcon 빌드 결과
├── cgps_pkg/ # GPS 수신 기능 패키지
├── dot_pkg/ # 도트/경로 시각화 기능
├── install/ # 빌드 후 설치 경로
├── log/ # 실행 중 생성된 로그
├── robot_action/ # ROS2 action 기능 정의
├── robot_cleaner_pkg/ # 장애물 회피 로직
├── robot_drive_pkg/ # 기본 주행 제어
├── robot_go_pkg/ # 목적지 이동 처리
├── robot_grid_pkg/ # 그리드 기반 경로 탐색
├── robot_move_pkg/ # 키보드 입력으로 로봇 제어 (teleop)
├── robot_total_pkg/ # 기능 통합 버전
├── robot_totalgrid_pkg/ # 좌표 기반 장애물 탐색 통합
├── robot_totalgrid_pkg_v2/ # 개선된 토탈그리드
├── test_pkg/ # 액션 서버/클라이언트 테스트
└── README.md # 프로젝트 설명 파일
```
---

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
  
10. ### cgps_pkg
   - EZ-0048 GPS 센서 드라이버 연동히여 위도·경도 정보를 터미널에 출력

11. ### dot_pkg
   - 일정 반경의 장애물을 도트 형식으로 시각화
---

## 🚀 설치 및 실행 방법
### 패키지 생성
```
cd robot_ws/src
ros2 pkg create 패키지명 --build-type ament_cmake --dependencies 필요 패키지
//자주 쓰는 패키지
ros2 pkg create 패키지명 --build-type ament_cmake --dependencies rclcpp std_msgs
```

### 패키지 빌드 및 실행
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
## ✅참고 문헌
TurtleBot3 공식 튜토리얼: http://emanual.robotis.com/docs/en/platform/turtlebot3/
ROS 2 튜토리얼: https://docs.ros.org/en/foxy/Tutorials.html

## 📄 논문
이 프로젝트 진행중 작성한 논문.
<<휴머노이드 로봇 제어를 위한 ROS 노드 설계>>
https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE11860907

## 🏆 공모전(캡스톤디자인)
지자체-대학 협력기반 지역혁신 사업 프로젝트랩 캡스톤디자인 - 의료기기를 위한 ROS2 포팅 프로젝트 랩

## 👩‍💻 작성자
안해은 외 4명
  
## 📜 라이선스
MIT License
