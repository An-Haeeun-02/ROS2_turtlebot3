#include <iostream>         // 입출력 관련 라이브러리
#include <vector>           // 벡터 자료구조를 사용하기 위한 라이브러리
#include <queue>            // 우선순위 큐 사용을 위한 라이브러리
#include <cmath>            // 수학 함수 사용을 위한 라이브러리 (sqrt, abs 등)
#include <rclcpp/rclcpp.hpp> // ROS2 노드를 생성하고 통신하는 라이브러리
#include <nav_msgs/msg/odometry.hpp> // ROS2에서 Odometry 메시지 타입을 사용하기 위한 헤더
#include <sensor_msgs/msg/laser_scan.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist.hpp" // 퍼블리셔(보내는 쪽) 메시지 타입
#include "geometry_msgs/msg/vector3.hpp" // 서브스크라이버(받는 쪽) 메시지 타입
#include <algorithm>        // 알고리즘 관련 함수 사용을 위한 라이브러리 (reverse 등)
#include <thread>           // 스레드 기능 사용을 위한 라이브러리
#include <csignal>          // 시그널 처리 (프로그램 종료 시그널 등) 사용을 위한 라이브러리

using namespace std;        // std 네임스페이스 사용

// 터미널 출력 시 색상 표현을 위한 ANSI 코드 정의
#define RESET "\033[0m"     
#define BLUE "\033[34m"     
#define RED "\033[31m"      
#define GREEN "\033[32m"    
#define YELLOW "\033[33m" 

class TurtlebotController : public rclcpp::Node {
public :
    // 생성자에서 Odometry 토픽을 구독하고, cmd_vel 퍼블리셔 생성
    TurtlebotController() : rclcpp::Node("turtlebot_controller"),
        obstacle_detected_(false),
        stair_detected_(false), 
        previous_left_value_(0.0), 
        previous_right_value_(0.0) {

        // Odometry 메시지 구독 설정
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&TurtlebotController::odom_callback, this, std::placeholders::_1));
        // cmd_vel 퍼블리셔 설정
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // 정면 센서 데이터 구독
        sensor_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("aeeun", 10,
            std::bind(&TurtlebotController::sensor_callback, this, std::placeholders::_1));
        // 우측 센서 데이터 구독
        sensor_right_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("hyewon", 10,
            std::bind(&TurtlebotController::sensor_right_callback, this, std::placeholders::_1));
        // 좌측 센서 데이터 구독
        sensor_left_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("dongwan", 10,
            std::bind(&TurtlebotController::sensor_left_callback, this, std::placeholders::_1));
        // IMU 센서 데이터 구독
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, 
            std::bind(&TurtlebotController::imuCallback, this, std::placeholders::_1));
        // 전방 센서 데이터 구독
        sensor_front_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("/haeeun", 10,
            std::bind(&TurtlebotController::sensor_front_callback, this, std::placeholders::_1));
    }
    // 좌표 저장을 위한 구조체
    struct GridPoint { 
        int x, y;
        GridPoint(int x = 0, int y = 0) : x(x), y(y) {}
        // 좌표 비교를 위한 연산자 오버로딩
        bool operator==(const GridPoint& other) const {
            return x == other.x && y == other.y;
        }
    };
    // 경로 설정 및 장애물 처리 함수
    void make_root() {
        int width, height;
        std::cout << "그리드 너비를 입력하세요: ";  // 사용자로부터 그리드 맵의 너비 입력받음
        std::cin >> width;
        std::cout << "그리드 높이를 입력하세요: ";  // 사용자로부터 그리드 맵의 높이 입력받음
        std::cin >> height;

        // 그리드 맵 초기화 (width가 열, height가 행으로 설정) 2차원 벡터(height x width)로 0으로 초기화
        grid_map = std::vector<std::vector<int>>(height, std::vector<int>(width, 0));

        // 시작점 입력
        std::cout << "시작 점 (x y)을 입력하세요: "; // 사용자로부터 시작 지점의 좌표 입력 받음
        std::cin >> start.x >> start.y;

        // 목표점 입력
        std::cout << "목표 점 (x y)을 입력하세요: "; // 사용자로부터 목표 지점의 좌표 입력 받음
        std::cin >> goal.x >> goal.y;

        // 장애물의 수 입력
        int num_obstacles;
        std::cout << "장애물의 수를 입력하세요: ";
        std::cin >> num_obstacles;

        // 사용자로부터 장애물의 위치와 크기 입력받아 벡터에 저장
        for (int i = 0; i < num_obstacles; ++i) {
            GridPoint obstacle;
            int obs_width, obs_height;
            std::cout << "장애물 " << i + 1 << "의 위치 (x y)를 입력하세요: "; // 장애물의 좌표 입력 받음
            std::cin >> obstacle.x >> obstacle.y;
            std::cout << "장애물 " << i + 1 << "의 너비와 높이를 입력하세요: "; // 장애물의 너비와 높이 입력 받음
            std::cin >> obs_width >> obs_height;

            obstacles.push_back(obstacle);           // 장애물 위치를 벡터에 저장
            obstacle_sizes.push_back({obs_width, obs_height}); // 장애물 크기 저장
        }

        // 장애물을 그리드 맵에 반영
        set_obstacles();

        // 경로 탐색 알고리즘 실행
        std::vector<GridPoint> path = find_path();

        // 경로가 발견되었을 때
        if (!path.empty()) {
            std::cout << "경로가 발견되었습니다:\n";
            print_grid_with_path(path); // 경로를 포함한 그리드 출력

            // 경로 거리 계산 및 출력
            double total_distance = calculate_path_distance(path);
            std::cout << "총 경로 거리: " << total_distance << " cm" << std::endl;
            std::cout << "목표 지점에 도달했습니다!" << std::endl;
        } else {
            // 경로를 찾지 못했을 때
            std::cout << "경로를 찾을 수 없습니다." << std::endl;
        }
    }

    // 로봇 이동 처리 함수
    void move_robot() {
        geometry_msgs::msg::Twist message;
        int cm = 10;  // 1 블록 당 거리 10cm로 설정
        cout << "로봇을 움직입니다." << endl;

        // 시작 지점부터 마지막 중간 지점까지 경로 처리
        for (int i = 1; i < setGridPoint.size() - 1; i++) {
            if (obstacle_detected_) {
            cout << "장애물 감지. 로봇을 멈춥니다." << endl;
            message.linear.x = 0.0; // 전진
            message.angular.z = 0.0; // 회전 없음
            cmd_publisher_->publish(message); // 명령 메시지 발행
            return;  // 함수 종료
            } else{
            cout << i - 1 << "번째 지점 -> " << i << "번째 지점" << endl;
            cout << setGridPoint[i][0] << "," << setGridPoint[i][1] << " -> " << setGridPoint[i + 1][0] << "," << setGridPoint[i + 1][1] << endl;
            // 경로 이동 처리
            process_path(setGridPoint[i][0], setGridPoint[i][1], setGridPoint[i + 1][0], setGridPoint[i + 1][1], cm);            
            }
        }

        // 마지막 중간 지점부터 도착 지점까지 경로 처리
        if (obstacle_detected_) {
            cout << "장애물 감지. 로봇을 멈춥니다." << endl;
            message.linear.x = 0.0; // 전진
            message.angular.z = 0.0; // 회전 없음
            cmd_publisher_->publish(message); // 명령 메시지 발행
            return;  // 함수 종료
        }else{
        cout << setGridPoint.size() - 2 << "번째 지점 -> " << "도착지" << endl;
        cout << setGridPoint[setGridPoint.size() - 1][0] << "," << setGridPoint[setGridPoint.size() - 1][1] << " -> " << setGridPoint[0][0] << "," << setGridPoint[0][1] << endl;
        // 경로 이동 처리
        process_path(setGridPoint[setGridPoint.size() - 1][0], setGridPoint[setGridPoint.size() - 1][1], setGridPoint[0][0], setGridPoint[0][1], cm);                    
        }

        cout << "도착했습니다." << endl;  // 도착 메시지 출력
    }

    // 로봇 제어 함수
    void robot_controller() {
        make_root(); // 경로 설정
        move_robot(); // 로봇 이동 처리
    }


private :
     // 멤버 변수 선언
    std::vector<std::vector<int>> grid_map; // 그리드 맵을 저장할 벡터
    std::vector<std::vector<int>> obstacle_sizes; // 장애물 크기를 저장할 벡터
    std::vector<GridPoint> obstacles;  // 장애물 위치를 저장할 벡터
    GridPoint start, goal;  // 시작점과 목표점
    double distance_traveled = 0.0;  // 누적 이동 거리
    std::vector<std::vector<int>> setGridPoint; // 회전 지점 저장
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // odom 토픽 구독자
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; // IMU 센서 데이터 구독
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_right_subscriber_; // 우측 센서 데이터 구독
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_left_subscriber_; // 좌측 센서 데이터 구독
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_front_subscriber_; // 전방 센서 데이터 구독
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_subscriber_; // 정면 센서 데이터 구독
    

    // 센서 데이터 저장용 변수
    geometry_msgs::msg::Vector3 sensor_right_data_; // 우측 센서 데이터 저장
    geometry_msgs::msg::Vector3 sensor_left_data_; // 좌측 센서 데이터 저장
    geometry_msgs::msg::Vector3 sensor_front_data_; // 전방 센서 데이터 저장
    geometry_msgs::msg::Vector3 sensor_data_; // 정면 센서 데이터 저장

    // 상태 및 초기화 관련 변수
    bool obstacle_detected_; // 장애물 감지 플래그
    double previous_left_value_; // 이전 왼쪽 센서 값 저장
    double previous_right_value_; // 이전 오른쪽 센서 값 저장
    double initial_orientation_; // 초기 방향(로봇의 처음 방향)
    double current_orientation_; // 현재 방향(회전 값을 포함한 방향)
    
    double tfmini_range_;  // tfmini 거리 측정값 저장
    bool stair_detected_;  // 계단 감지 여부 플래그
    
    bool is_initialized_ = false; // 초기화 여부 체크 플래그
    bool stop_requested_ = false; // 중지 요청 플래그

    std::map<int, double> sensor_thresholds_; // 각도별 센서값 임계값


    // 방향을 나타내는 열거형
    enum Direction {
        UP, RIGHT, DOWN, LEFT
    };
     // A* 알고리즘을 위한 노드 구조체
    struct Node {
        GridPoint point;  // 좌표
        Direction dir;  // 방향
        int g, h;  // g: 시작점부터의 비용, h: 휴리스틱 비용
        Node* parent;  // 부모 노드 포인터(이전 좌표)

        Node(GridPoint p, Direction d, int g, int h, Node* parent = nullptr) 
            : point(p), dir(d), g(g), h(h), parent(parent) {}
        int f() const { return g + h; }  // f = 출발장애물거리 + 장애물목표거리

        bool operator>(const Node& other) const { // 우선순위 큐에서 사용하기 위한 연산자
            return f() > other.f();
        }
    };
    // 휴리스틱 함수 (두 점 사이의 맨해튼 거리 계산)
    int heuristic(const GridPoint& a, const GridPoint& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

// A* 알고리즘으로 경로 탐색하는 함수
    std::vector<GridPoint> find_path() {
        // 우선순위 큐 (A* 알고리즘에서 사용)
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list; 
        // 탐색된 노드를 저장하는 리스트
        std::vector<std::vector<bool>> closed_list(grid_map.size(), std::vector<bool>(grid_map[0].size(), false)); 

        // 시작점을 우선순위 큐에 넣음
        open_list.push(Node(start, RIGHT, 0, heuristic(start, goal)));

        std::vector<GridPoint> turn_GridPoints; // 회전 지점을 저장할 벡터

        // 경로 탐색 루프
        while (!open_list.empty()) {
            Node current = open_list.top(); // 우선순위가 가장 높은 노드를 꺼냄
            open_list.pop();

            // 목표 지점에 도달하면 경로를 반환
            if (current.point == goal) {
                std::vector<GridPoint> path;
                while (current.parent) {
                    // 현재 노드를 경로에 추가
                    path.push_back(current.point);
                    // 세 노드가 직선상에 있지 않다면 회전이 필요하므로 회전 지점으로 기록
                    if (current.parent && is_turn(current.parent->point, current.point, path.back()))  {
                        // 회전 지점으로 판별된 노드롤 turn_GridPoints 벡터에 저장
                        turn_GridPoints.push_back(current.point);
                    } 
                    // 현재 노드를 부모 노드로
                    current = *current.parent;
                }

                // 마지막으로 시작점도 경로에 추가
                path.push_back(start); 
                // 경로를 뒤집어 올바른 순서로 변환
                std::reverse(path.begin(), path.end());

                // 회전 지점 설정
                setGridPoint.resize(2 + turn_GridPoints.size(), std::vector<int>(2));
                setGridPoint[1][0] = start.x; setGridPoint[1][1] = start.y;
                setGridPoint[0][0] = goal.x; setGridPoint[0][1] = goal.y;
                for (size_t i = 0; i < turn_GridPoints.size(); ++i) {
                    setGridPoint[2 + i][0] = turn_GridPoints[i].x;
                    setGridPoint[2 + i][1] = turn_GridPoints[i].y;
                }

                return path;  // 경로 반환
            }

            // 현재 노드를 탐색 완료로 표시
            closed_list[current.point.y][current.point.x] = true; 

            // 이동 가능한 방향 정의
            std::vector<std::pair<GridPoint, Direction>> directions = {
                {{1, 0}, RIGHT}, // X축 방향으로 +1
                {{0, 1}, DOWN}, 
                {{-1, 0}, LEFT}, 
                {{0, -1}, UP}
            };

            // 각 방향으로 이동 가능한지 확인
            for (const auto& [dir, direction] : directions) {
                GridPoint next(current.point.x + dir.x, current.point.y + dir.y); 

                // 경계 및 장애물 체크
                if (next.x >= 0 && next.x < static_cast<int>(grid_map[0].size()) &&
                    next.y >= 0 && next.y < static_cast<int>(grid_map.size()) &&
                    grid_map[next.y][next.x] == 0 && !closed_list[next.y][next.x]) {

                    int rotation_cost = (current.dir == direction) ? 0 : 1; // 회전 비용 계산
                    // 새로운 노드를 우선순위 큐에 추가
                    open_list.push(Node(next, direction, current.g + 1 + rotation_cost, heuristic(next, goal), new Node(current))); 
                }
            }
        }
        return {}; // 경로를 찾지 못하면 빈 벡터 반환
    }
    // 장애물을 그리드 맵에 설정하는 함수
    void set_obstacles() {
        // 각 장애물을 그리드 맵에 반영
        for (size_t i = 0; i < obstacles.size(); ++i) {
            GridPoint obs = obstacles[i]; 
            int obs_width = obstacle_sizes[i][0]; 
            int obs_height = obstacle_sizes[i][1]; 

            // 장애물이 그리드의 여러 칸을 차지할 수 있으므로 범위 설정
            for (size_t x = static_cast<size_t>(obs.y); x < static_cast<size_t>(obs.y) + static_cast<size_t>(obs_height) && x < grid_map.size(); ++x) {
                for (size_t y = static_cast<size_t>(obs.x); y < static_cast<size_t>(obs.x) + static_cast<size_t>(obs_width) && y < grid_map[0].size(); ++y) {
                    grid_map[x][y] = 1; // 장애물 있는 칸을 1로 표시
                }
            }
        }
    }

    // 경로를 그리드 맵에 출력하는 함수
    void print_grid_with_path(const std::vector<GridPoint>& path) {
        // 표시용 그리드 맵 초기화 (빈 칸은 ' ')
        std::vector<std::vector<char>> display_map(grid_map.size(), std::vector<char>(grid_map[0].size(), ' ')); 

        // 장애물을 표시
        for (size_t i = 0; i < obstacles.size(); ++i) {
            GridPoint obs = obstacles[i];
            int obs_width = obstacle_sizes[i][0];
            int obs_height = obstacle_sizes[i][1];

            for (size_t x = static_cast<size_t>(obs.y); x < static_cast<size_t>(obs.y) + static_cast<size_t>(obs_height) && x < display_map.size(); ++x) {
                for (size_t y = static_cast<size_t>(obs.x); y < static_cast<size_t>(obs.x) + static_cast<size_t>(obs_width) && y < display_map[0].size(); ++y) {
                    display_map[x][y] = '#'; // 장애물은 '#'으로 표시
                }
            }
        }

        // 경로 중 회전 지점을 추가적으로 표시
        std::vector<GridPoint> new_turn_GridPoints;
        for (size_t i = 1; i < path.size() - 1; ++i) {
            if (is_turn(path[i - 1], path[i], path[i + 1])) {
                new_turn_GridPoints.push_back(path[i]);
            }
        }

        // 회전 지점을 표시할 setGridPoint 배열 설정
        setGridPoint.resize(2 + new_turn_GridPoints.size(), std::vector<int>(2));
        setGridPoint[1][0] = start.x; setGridPoint[1][1] = start.y;
        setGridPoint[0][0] = goal.x; setGridPoint[0][1] = goal.y;
        for (size_t i = 0; i < new_turn_GridPoints.size(); ++i) {
            setGridPoint[2 + i][0] = new_turn_GridPoints[i].x;
            setGridPoint[2 + i][1] = new_turn_GridPoints[i].y;
        }

        // 회전 지점을 표시
        for (const auto& tp : new_turn_GridPoints) {
            display_map[tp.y][tp.x] = '+'; // 회전 지점은 '+'로 표시
        }

        // 시작점과 목표점 표시
        display_map[start.y][start.x] = '*'; // 시작점 표시
        display_map[goal.y][goal.x] = '*'; // 목표점 표시

        size_t width = display_map[0].size();
        size_t height = display_map.size();

        // 상단의 테두리 '@' 출력
        for (size_t i = 0; i < width + 2; ++i) {
            std::cout << "@ ";
        }
        std::cout << std::endl;

        // 각 행을 출력
        for (size_t i = 0; i < height; ++i) {
            std::cout << "@ "; // 왼쪽 테두리
            for (size_t j = 0; j < width; ++j) {
                if (display_map[i][j] == '#') { // 장애물 파란색
                    std::cout << BLUE << display_map[i][j] << RESET << ' '; 
                } else if (display_map[i][j] == '+') { // 회전 지점 노란색
                    std::cout << YELLOW << display_map[i][j] << RESET << ' '; 
                } else if (i == static_cast<size_t>(start.y) && j == static_cast<size_t>(start.x)) { // 시작점 빨간색
                    std::cout << RED << display_map[i][j] << RESET << ' '; 
                } else if (i == static_cast<size_t>(goal.y) && j == static_cast<size_t>(goal.x)) { // 목표점 초록색
                    std::cout << GREEN << display_map[i][j] << RESET << ' '; 
                } else {
                    std::cout << display_map[i][j] << ' '; // 빈 칸 기본 색상
                }
            }
            std::cout << "@ " << std::endl; // 오른쪽 테두리
        }

        // 하단의 테두리 '@' 출력
        for (size_t i = 0; i < width + 2; ++i) {
            std::cout << "@ ";
        }
        std::cout << std::endl;

        // 회전 지점 좌표 출력
        std::cout << "회전 지점 좌표들:" << std::endl;
        for (const auto& row : setGridPoint) {
            std::cout << "(" << row[0] << ", " << row[1] << ")" << std::endl;
        }
    }

    // 경로의 총 이동 거리 계산
    double calculate_path_distance(const std::vector<GridPoint>& path) {
        double total_distance = 0.0; // 총 거리
        for (size_t i = 1; i < path.size(); ++i) {
            double dx = path[i].x - path[i - 1].x;
            double dy = path[i].y - path[i - 1].y;
            // 유클리드 거리 계산
            total_distance += std::sqrt(dx * dx + dy * dy) * 10; // 각 좌표 사이의 거리 계산
        }
        return total_distance; 
    };

    // 회전 여부 판단 함수 (세 점이 직선상에 있는지 확인)
    bool is_turn(const GridPoint& prev, const GridPoint& , const GridPoint& next) {
        return (prev.x != next.x && prev.y != next.y);
    }

/*이동 함수*/
// 경로 이동 처리 함수 (x 또는 y 좌표가 같을 때)
    void process_path(int x1, int y1, int x2, int y2, int cm) {
        int block;
        
        // x 좌표가 같은 경우
        if (x1 == x2) {
            block = abs(y2 - y1);  // y 좌표 차이 계산
            go_straight(block, cm);  // 직진
        }
        // y 좌표가 같은 경우
        else if (y1 == y2) {
            if (x1 > x2) {
                block = abs(x1-x2);  // x 좌표 차이 계산
                left_course(block, cm);  // 좌회전 후 직진
            } else {
                block = abs(x2 - x1);  // x 좌표 차이 계산
                right_course(block, cm);  // 우회전 후 직진
            }
        }
    }
    // 우회전 후 직진 함수
    void right_course(int block, int cm) {
        cout << "Turning right" << endl;
        right_turn();  // 우회전
        go_straight(block, cm);  // 직진
        left_turn();  // 좌회전으로 방향 복귀
    }
    // 좌회전 후 직진 함수
    void left_course(int block, int cm) {
        cout << "Turning left" << endl;
        left_turn();  // 좌회전
        go_straight(block, cm);  // 직진
        right_turn();  // 우회전으로 방향 복귀
    }
    // 우회전 함수 (IMU 기반)
    void right_turn() {
        double target_orientation = current_orientation_ - M_PI / 2.0; // 목표 방향: 현재 방향에서 -90도 (라디안)
        
        // 목표 각도가 -π 이하로 내려가면 +2π 해서 범위 내로 설정
        if (target_orientation < -M_PI) {
            target_orientation += 2 * M_PI;
        }

        auto message = geometry_msgs::msg::Twist();
        message.angular.z = -0.2; // 우회전 속도 설정

        // 목표 각도에 도달할 때까지 회전
        while (rclcpp::ok() && std::fabs(current_orientation_ - target_orientation) > 0.05) {
            cmd_publisher_->publish(message); // 회전 명령 퍼블리시
            std::this_thread::sleep_for(0.1s); // 0.1초 대기
        }

        // 정지 명령
        message.angular.z = 0.0;
        cmd_publisher_->publish(message);

        // 새로운 방향을 초기화
        initial_orientation_ = current_orientation_; // 회전 후 새로운 방향을 초기화
    }

    // 좌회전 함수 (IMU 기반)
    void left_turn() {
        double target_orientation = current_orientation_ + M_PI / 2.0; // 목표 방향: 현재 방향에서 +90도 (라디안)
        
        // 목표 각도가 π 이상으로 넘어가면 -2π 해서 범위 내로 설정
        if (target_orientation > M_PI) {
            target_orientation -= 2 * M_PI;
        }

        auto message = geometry_msgs::msg::Twist();
        message.angular.z = 0.2; // 좌회전 속도 설정

        // 목표 각도에 도달할 때까지 회전
        while (rclcpp::ok() && std::fabs(current_orientation_ - target_orientation) > 0.05) {
            cmd_publisher_->publish(message); // 회전 명령 퍼블리시
            std::this_thread::sleep_for(0.1s); // 0.1초 대기
        }

        // 정지 명령
        message.angular.z = 0.0;
        cmd_publisher_->publish(message);

        // 새로운 방향을 초기화
        initial_orientation_ = current_orientation_; // 회전 후 새로운 방향을 초기화
    }

    // 직진 함수 (주어진 거리만큼 직진)
    void go_straight(int block, double cm) {
        cout << "직진"  << endl;
        double target_distance = block * cm; // 블록 수에 따라 이동해야 할 거리 계산
        RCLCPP_INFO(this->get_logger(), "이동해야할 거리= %.2f cm", target_distance);
        
        distance_traveled = 0.0; // 이동 거리 초기화

        // cmd_vel 메시지 생성 (직진 속도 설정)
        geometry_msgs::msg::Twist message;
        message.linear.x = 0.1;  // 직진 속도 설정
        message.angular.z = 0.0; // 회전 속도는 0

        // 이동 중 거리 비교
        while (rclcpp::ok() && distance_traveled < target_distance) {
            if (obstacle_detected_) { // 장애물 감지 시
                cout << "장애물을 감지 했습니다. 전진을 정지 합니다." << endl;
                message.linear.x = 0.0; // 전진 멈춤
                cmd_publisher_->publish(message); // 정지 명령 발행
                this_thread::sleep_for(0.1s);
                break;
            } else { // 장애물 감지되지 않으면 계속 전진
            //RCLCPP_INFO(this->get_logger(), "이동해야할 거리= %.2f cm / 이동 한 거리 = %.2f / 남은거리 = %.2f", 
                                            //target_distance, distance_traveled * 100.0, target_distance - distance_traveled);
            cmd_publisher_->publish(message);  // 속도 명령을 퍼블리시
            controlLoop();//직진 각도 보정
            this_thread::sleep_for(0.5s); // 0.5초 대기
            }
        }

        // 정지 명령
        message.linear.x = 0.0;  // 정지
        cmd_publisher_->publish(message);  // 정지 명령 퍼블리시
    }

/*콜백함수*/
    // Odometry 콜백 함수 (로봇의 현재 위치를 이용해 이동 거리를 계산)
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        static double prev_x = 0.0, prev_y = 0.0; // 이전 위치 저장 변수
        
        double current_x = msg->pose.pose.position.x * 100.0; // 현재 x 좌표
        double current_y = msg->pose.pose.position.y * 100.0; // 현재 y 좌표
        
        double dx = current_x - prev_x;  // x 좌표 변화량
        double dy = current_y - prev_y;  // y 좌표 변화량
        double delta_distance = std::sqrt(dx * dx + dy * dy); // 이동 거리 계산
        
        distance_traveled += delta_distance;  // 누적 이동 거리 업데이트

        prev_x = current_x; // 이전 x 좌표 업데이트
        prev_y = current_y; // 이전 y 좌표 업데이트
    }
    // 정면 센서 콜백: 데이터를 저장하고, 임계값 및 장애물 확인
    void sensor_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        sensor_data_ = *msg; // 정면 센서 데이터 업데이트
        //check_sensor_threshold(); // 센서 임계값 확인
        check_obstacle(); // 장애물 확인
    }
    // 우측 센서 콜백: 데이터를 저장
    void sensor_right_callback(const geometry_msgs::msg::Vector3::SharedPtr rmsg) {
        sensor_right_data_ = *rmsg; // 우측 센서 데이터 업데이트
    }

    // 좌측 센서 콜백: 데이터를 저장
    void sensor_left_callback(const geometry_msgs::msg::Vector3::SharedPtr lmsg) {
        sensor_left_data_ = *lmsg; // 좌측 센서 데이터 업데이트
    }

    // 전방 센서 콜백: 데이터를 저장하고 계단 여부 확인
    void sensor_front_callback(const geometry_msgs::msg::Vector3::SharedPtr fmsg) {
        sensor_front_data_ = *fmsg; // 전방 센서 데이터 업데이트
        //check_stairs();  // 계단 여부 확인
    }
    // IMU 콜백 함수: 로봇의 현재 방향을 업데이트
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
        tf2::Quaternion quat; // 쿼터니언 데이터 받기
        tf2::fromMsg(msg->orientation, quat); // IMU 메시지로부터 쿼터니언으로 변환
        double roll, pitch, yaw; // 회전 각도(롤, 피치, 요)
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 쿼터니언을 롤, 피치, 요 각도로 변환

        current_orientation_ = yaw; // 현재 방향 업데이트

        if (!is_initialized_) {
          initial_orientation_ = yaw;  // 초기 방향 설정
          is_initialized_ = true; // 초기화 완료 플래그 설정
        }
    }

/*장애물 감지*/
    // 장애물 확인 함수: 정면 센서 값이 특정 값 이하일 때 장애물 감지
    void check_obstacle() {
        obstacle_detected_ = sensor_data_.x < 30.0; // 장애물 감지 여부 확인
    }
    /* 장애물 좌표 계산 및 판단 */
void detect_obstacle_position() {
    // 로봇이 장애물을 감지한 좌표를 구하기 위해 현재 위치와 방향을 기준으로 장애물 좌표를 계산합니다.
    int obstacle_x = robot_x_;
    int obstacle_y = robot_y_;
    int obstacle_distance = 1;  // 장애물 감지 거리를 1칸으로 가정합니다. 실제 센서 데이터에 따라 조정하세요.

    // 로봇의 현재 방향에 따라 장애물 좌표 계산
    switch (robot_direction_) {
        case UP:
            obstacle_y -= obstacle_distance; // 위쪽으로 한 칸 이동한 곳에 장애물
            break;
        case DOWN:
            obstacle_y += obstacle_distance; // 아래쪽으로 한 칸 이동한 곳에 장애물
            break;
        case LEFT:
            obstacle_x -= obstacle_distance; // 왼쪽으로 한 칸 이동한 곳에 장애물
            break;
        case RIGHT:
            obstacle_x += obstacle_distance; // 오른쪽으로 한 칸 이동한 곳에 장애물
            break;
    }

    // 장애물 좌표 출력
    std::cout << "장애물 감지 위치: (" << obstacle_x << ", " << obstacle_y << ")" << std::endl;

    // 미리 입력된 장애물인지 확인
    bool is_existing_obstacle = false;
    for (size_t i = 0; i < obstacles.size(); ++i) {
        int obs_x = obstacles[i].x;
        int obs_y = obstacles[i].y;
        int obs_width = obstacle_sizes[i][0];
        int obs_height = obstacle_sizes[i][1];

        // 장애물이 그리드의 여러 칸을 차지하므로 범위 내에 있는지 확인
        if (obstacle_x >= obs_x && obstacle_x < obs_x + obs_width &&
            obstacle_y >= obs_y && obstacle_y < obs_y + obs_height) {
            is_existing_obstacle = true;
            break;
        }
    }

    // 기존 장애물인지 새로운 장애물인지 출력
    if (is_existing_obstacle) {
        std::cout << "기존 장애물입니다." << std::endl;
    } else {
        std::cout << "새로운 장애물입니다." << std::endl;
    }
}


/*로봇 제어 관련 함수*/
    // 로봇 제어 루프: 각도를 조정하여 로봇을 직진시킴
    void controlLoop(){
        double angle_diff = current_orientation_ - initial_orientation_; // 현재 각도와 초기 각도의 차이 계산
        cout << "직진 각도를 수정합니다." << endl; 

        // 각도 차이를 [-π, π] 범위로 정규화
        while (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI)
            angle_diff += 2 * M_PI;

        auto message = geometry_msgs::msg::Twist(); // Twist 메시지 생성
        message.linear.x = 0.2;  // 기본 직진 속도 설정

        // 각도 차이가 0.05 라디안 이상일 때 보정 회전
        if (std::fabs(angle_diff) > 0.05) {
            message.angular.z = -angle_diff * 1.0;  // 각도 차이에 비례하여 회전 속도 설정
            message.linear.x = 0.1;  // 회전 중일 때는 직진 속도 줄임
        } else {
            message.angular.z = 0.0;  // 각도 차이가 작으면 회전하지 않음
        }

        // 퍼블리셔를 통해 명령 발행
        cmd_publisher_->publish(message);
    

}

};
// 종료 시그널 처리 함수 (Ctrl+C로 프로그램을 안전하게 종료)
void signalHandler(int /*signum*/) {
    cout << "Ctrl+C로 종료합니다." << endl;

    // 로봇 정지 명령을 퍼블리시
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0; 
    message.angular.z = 0.0; 

    auto node = rclcpp::Node::make_shared("stop_publisher"); // 퍼블리셔 노드 생성
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // cmd_vel 퍼블리셔 생성
    pub->publish(message); // 정지 명령 퍼블리시

    rclcpp::shutdown(); // ROS2 시스템 종료
}

// 메인 함수
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); // ROS2 시스템 초기화

    auto node = std::make_shared<TurtlebotController>(); // TurtlebotController 노드 생성

    // 로봇 제어 함수 실행을 위한 스레드 생성
    std::thread control_thread(&TurtlebotController::robot_controller, node); 

    signal(SIGINT, signalHandler); // Ctrl+C 시그널 처리 설정

    rclcpp::spin(node); // ROS2 메시지 처리

    control_thread.join(); // 스레드 종료 대기

    return 0; 
}