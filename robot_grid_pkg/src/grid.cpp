#include <iostream>         // 입출력 관련 라이브러리
#include <vector>           // 벡터 자료구조를 사용하기 위한 라이브러리
#include <queue>            // 우선순위 큐 사용을 위한 라이브러리
#include <cmath>            // 수학 함수 사용을 위한 라이브러리 (sqrt, abs 등)
#include <rclcpp/rclcpp.hpp> // ROS2 노드를 생성하고 통신하는 라이브러리
#include <nav_msgs/msg/odometry.hpp> // ROS2에서 Odometry 메시지 타입을 사용하기 위한 헤더
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

// TurtlebotController 클래스 정의 (ROS2 노드 상속)
class TurtlebotController : public rclcpp::Node {
public:
    TurtlebotController() : rclcpp::Node("turtlebot_controller") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&TurtlebotController::odom_callback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    void make_root() {
        int width, height;
        std::cout << "그리드 너비를 입력하세요: ";  // 너비를 먼저 입력 받음
        std::cin >> width;
        std::cout << "그리드 높이를 입력하세요: ";  // 높이를 나중에 입력 받음
        std::cin >> height;

        // 그리드 맵 초기화 (width가 열, height가 행으로 설정) 2차원 벡터(height x width)
        grid_map = std::vector<std::vector<int>>(height, std::vector<int>(width, 0));

        // 시작점 입력
        std::cout << "시작 점 (x y)을 입력하세요: "; // x y 순서로 입력 받음
        std::cin >> start.x >> start.y;

        // 목표점 입력
        std::cout << "목표 점 (x y)을 입력하세요: "; // x y 순서로 입력 받음
        std::cin >> goal.x >> goal.y;

        // 장애물의 수 입력
        int num_obstacles;
        std::cout << "장애물의 수를 입력하세요: ";
        std::cin >> num_obstacles;

        // 장애물 정보 입력 (위치와 크기)
        for (int i = 0; i < num_obstacles; ++i) {
            Point obstacle;
            int obs_width, obs_height;
            std::cout << "장애물 " << i + 1 << "의 위치 (x y)를 입력하세요: "; // x y 순서로 입력 받음
            std::cin >> obstacle.x >> obstacle.y;
            std::cout << "장애물 " << i + 1 << "의 너비와 높이를 입력하세요: "; // 너비와 높이 입력 순서
            std::cin >> obs_width >> obs_height;

            obstacles.push_back(obstacle);           // 장애물 위치 저장
            obstacle_sizes.push_back({obs_width, obs_height}); // 장애물 크기 저장
        }

        // 장애물을 그리드 맵에 설정
        set_obstacles();

        // 경로 탐색
        std::vector<Point> path = find_path();

        // 경로가 발견되었을 때
        if (!path.empty()) {
            std::cout << "경로가 발견되었습니다:\n";
            print_grid_with_path(path); // 경로를 포함한 그리드 출력

            // 경로 거리 계산 및 출력
            double total_distance = calculate_path_distance(path);
            std::cout << "총 경로 거리: " << total_distance << " cm" << std::endl;
            std::cout << "목표 지점에 도달했습니다!" << std::endl;
        } else {
            std::cout << "경로를 찾을 수 없습니다." << std::endl; // 경로를 찾지 못했을 때 메시지 출력
        }
    }

    void move_robot() {
        int block;
        int cm = 10;
        cout << "로봇을 움직입니다." << endl;
        
        // 시작 지점과 첫 중간 지점 사이의 경로 처리
        cout << "시작->1번째 지점" << endl;
        cout << setPoint[0][0] << "," << setPoint[0][1] << " -> " << setPoint[2][0] << "," << setPoint[2][1] << endl;
        if (setPoint[0][0] == setPoint[2][0]) { // x가 같을 때
            block = setPoint[2][1] - setPoint[0][1];//y비교
            go_straight(block, cm);
        } else if (setPoint[0][1] == setPoint[2][1]) { // y가 같을 때
            if (setPoint[0][0] > setPoint[2][0]) {//x비교
                block = setPoint[0][0] - setPoint[2][0];
                left_course(block, cm); // 좌회전 후 직진
            } else if (setPoint[0][0] < setPoint[2][0]) {
                block = setPoint[2][0] - setPoint[0][0];
                right_course(block, cm); // 우회전 후 직진
            }
        }
        
        // 두 번째 중간 지점부터
        for (int i = 2; setPoint.size() - 1 > i; i++) {
            cout << i - 1 << "번째 지점 -> " << i << "번째 지점" << endl;
            cout << setPoint[i][0] << "," << setPoint[i][1] << " -> " << setPoint[i + 1][0] << "," << setPoint[i + 1][1] << endl;
            if (setPoint[i][0] == setPoint[i + 1][0]) { // x가 같을 때
                if (setPoint[i + 1][1] < setPoint[i][1]) {//y값 비교
                    block = setPoint[i][1] - setPoint[i + 1][1];
                } else {
                    block = setPoint[i + 1][1] - setPoint[i][1];  
                }
                go_straight(block, cm);
            } else if (setPoint[i][1] == setPoint[i + 1][1]) { // y가 같을 때
                if (setPoint[i][0] > setPoint[i + 1][0]) {//x 값 비교
                    block = setPoint[i][0] - setPoint[i + 1][0];
                    left_course(block, cm); // 좌회전 후 직진
                } else if (setPoint[i][0] < setPoint[i + 1][0]) {
                    block = setPoint[i + 1][0] - setPoint[i][0];
                    right_course(block, cm); // 우회전 후 직진
                }
            }
        }

        // 마지막 중간 지점부터 도착 지점까지
        cout << setPoint.size() - 2 << "번째 지점 -> " << "도착지" << endl;
        cout << setPoint[setPoint.size() - 1][0] << "," << setPoint[setPoint.size() - 1][1] << " -> " << setPoint[1][0] << "," << setPoint[1][1] << endl;
        if (setPoint[setPoint.size() - 1][0] == setPoint[1][0]) { // x가 같을 때
            block = setPoint[1][1] - setPoint[setPoint.size() - 1][1];//y비교
            go_straight(block, cm);
        } else if (setPoint[setPoint.size() - 1][1] == setPoint[1][1]) { // y가 같을 때
            if (setPoint[setPoint.size() - 1][0] > setPoint[0][0]) {//x비교
                block = setPoint[setPoint.size() - 1][0] - setPoint[1][0];
                left_course(block, cm); // 좌회전 후 직진
            } else if (setPoint[setPoint.size() - 1][0] < setPoint[1][0]) {
                block = setPoint[1][0] - setPoint[setPoint.size() - 1][0];
                right_course(block, cm); // 우회전 후 직진
            }
        }
        cout << "도착했습니다." << endl;
    }

    void go_straight(int block, double cm) {
        cout << "직진"  << endl;
        double target_distance = block * cm; // 이동해야 하는 거리
        RCLCPP_INFO(this->get_logger(), "이동해야할 거리= %.2f cm", target_distance);
        
        distance_traveled = 0.0; // 이동 거리 초기화

        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = 0.1;  // 직진 속도 설정
        cmd_msg.angular.z = 0.0; // 회전 속도는 0

        while (rclcpp::ok() && distance_traveled < target_distance) {
            RCLCPP_INFO(this->get_logger(), "이동해야할 거리= %.2f cm / 이동 한 거리 = %.2f / 남은거리 = %.2f", 
                                            target_distance, distance_traveled * 100.0, target_distance - distance_traveled);
            cmd_publisher_->publish(cmd_msg);  // 속도 명령을 퍼블리시
            this_thread::sleep_for(0.5s);
        }

        cmd_msg.linear.x = 0.0;  // 정지
        cmd_publisher_->publish(cmd_msg);  // 정지 명령 퍼블리시
    }

    void right_course(int block, int cm) {
        cout << "Turning right" << endl;
        right_turn();
        go_straight(block, cm);
        left_turn();
    }

    void left_course(int block, int cm) {
        cout << "Turning left" << endl;
        left_turn();
        go_straight(block, cm);
        right_turn();
    }

    void right_turn() {
        auto message = geometry_msgs::msg::Twist();
        message.angular.z = -3.14 / 4.0; // 오른쪽으로 90도 회전
        cmd_publisher_->publish(message);
        std::this_thread::sleep_for(std::chrono::seconds(2)); // 회전 대기
        message.angular.z = 0.0; // 회전 멈춤
        cmd_publisher_->publish(message);
    }

    void left_turn() {
        auto message = geometry_msgs::msg::Twist();
        message.angular.z = 3.14 / 4.0; // 왼쪽으로 90도 회전
        cmd_publisher_->publish(message);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        message.angular.z = 0.0; // 회전 멈춤
        cmd_publisher_->publish(message);
    }

    void robot_controller() {
        make_root(); // 먼저 make_root 실행
        move_robot(); // make_root가 완료되면 move_robot 실행
    }

private:
    struct Point { // point는 좌표 저장하는 구조체
        int x, y;
        Point(int x = 0, int y = 0) : x(x), y(y) {}
        bool operator==(const Point& other) const {
            return x == other.x && y == other.y;
        }
    };

    enum Direction {
        UP, RIGHT, DOWN, LEFT
    };

    std::vector<std::vector<int>> grid_map; // 그리드 맵을 저장할 벡터
    std::vector<std::vector<int>> obstacle_sizes; // 장애물 크기를 저장할 벡터
    std::vector<Point> obstacles;  // 장애물 위치를 저장할 벡터
    Point start, goal;  // 시작점과 목표점
    double distance_traveled = 0.0;  // 누적 이동 거리
    std::vector<std::vector<int>> setPoint; // 회전 지점 저장
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // odom 토픽 구독자
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    // 회전 여부 판단
    bool is_turn(const Point& prev, const Point& current, const Point& next) {
        return (prev.x != next.x && prev.y != next.y);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        static double prev_x = 0.0, prev_y = 0.0;
        
        double current_x = msg->pose.pose.position.x * 100.0;
        double current_y = msg->pose.pose.position.y * 100.0;
        
        double dx = current_x - prev_x;
        double dy = current_y - prev_y;
        double delta_distance = std::sqrt(dx * dx + dy * dy);
        
        distance_traveled += delta_distance;

        prev_x = current_x;
        prev_y = current_y;
    }

    struct Node {
        Point point;  // 좌표
        Direction dir;  // 방향
        int g, h;  // g: 시작점부터의 비용, h: 휴리스틱 비용
        Node* parent;  // 부모 노드 포인터(이전 좌표)

        Node(Point p, Direction d, int g, int h, Node* parent = nullptr) 
            : point(p), dir(d), g(g), h(h), parent(parent) {}
        int f() const { return g + h; }  // f = 출발장애물거리 + 장애물목표거리

        bool operator>(const Node& other) const {
            return f() > other.f();
        }
    };

    // 거리 계산
    int heuristic(const Point& a, const Point& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    std::vector<Point> find_path() {
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list; 
        std::vector<std::vector<bool>> closed_list(grid_map.size(), std::vector<bool>(grid_map[0].size(), false)); 

        open_list.push(Node(start, RIGHT, 0, heuristic(start, goal)));

        std::vector<Point> turn_points; 

        while (!open_list.empty()) {
            Node current = open_list.top(); 
            open_list.pop();

            // 현재지점이 목표지점이면 path에 저장
            if (current.point == goal) {
                std::vector<Point> path;
                while (current.parent) {
                    // 현재 노드를 경로에 추가
                    path.push_back(current.point);
                    // 세 노드가 직선상에 있지 않다면 회전이 필요하므로 회전 지점으로 기록
                    if (current.parent && is_turn(current.parent->point, current.point, path.back()))  {
                    // 회전 지점으로 판별된 노드롤 turn_points 벡터에 저장
                        turn_points.push_back(current.point);
                    } 
                    // 현재 노드를 부모 노드로
                    current = *current.parent;
                }

                // 마지막으로 시작점도 경로에 추가
                path.push_back(start); 
                // 경로를 뒤집어 올바른 순서로 변환(..)
                std::reverse(path.begin(), path.end());

                //..
                setPoint.resize(2 + turn_points.size(), std::vector<int>(2));
                setPoint[0][0] = start.x; setPoint[0][1] = start.y;
                setPoint[1][0] = goal.x; setPoint[1][1] = goal.y;
                for (size_t i = 0; i < turn_points.size(); ++i) {
                    setPoint[2 + i][0] = turn_points[i].x;
                    setPoint[2 + i][1] = turn_points[i].y;
                }

                return path;
            }

            // 각 노드가 탐색되었는지 확인
            closed_list[current.point.y][current.point.x] = true; 

            // 이동방향 정의
            std::vector<std::pair<Point, Direction>> directions = {
                {{1, 0}, RIGHT}, // X축 방향으로 +1
                {{0, 1}, DOWN}, 
                {{-1, 0}, LEFT}, 
                {{0, -1}, UP}
            };

            // 방향 체크?
            for (const auto& [dir, direction] : directions) {
                Point next(current.point.x + dir.x, current.point.y + dir.y); 

                // 경계 및 장애물 체크(..)
                if (next.x >= 0 && next.x < static_cast<int>(grid_map[0].size()) &&
                    next.y >= 0 && next.y < static_cast<int>(grid_map.size()) &&
                    grid_map[next.y][next.x] == 0 && !closed_list[next.y][next.x]) {

                    int rotation_cost = (current.dir == direction) ? 0 : 1; 
                    open_list.push(Node(next, direction, current.g + 1 + rotation_cost, heuristic(next, goal), new Node(current))); 
                }
            }
        }
        return {}; 
    }

    // 장애물을 그리드 맵에 설정
    void set_obstacles() {
        for (size_t i = 0; i < obstacles.size(); ++i) {
            Point obs = obstacles[i]; 
            int obs_width = obstacle_sizes[i][0]; 
            int obs_height = obstacle_sizes[i][1]; 

            // 장애물이 몇 칸의 공간을 차지하는지 확인 
            // 장애물이 세로 방향에 얼마나 많은 칸 차지
            for (size_t x = static_cast<size_t>(obs.y); x < static_cast<size_t>(obs.y) + static_cast<size_t>(obs_height) && x < grid_map.size(); ++x) {
                // 장애물이 가로 방향에 얼마나 많이 차지하는지
                for (size_t y = static_cast<size_t>(obs.x); y < static_cast<size_t>(obs.x) + static_cast<size_t>(obs_width) && y < grid_map[0].size(); ++y) {
                    grid_map[x][y] = 1; 
                }
            }
        }
    }

    void print_grid_with_path(const std::vector<Point>& path) {
        std::vector<std::vector<char>> display_map(grid_map.size(), std::vector<char>(grid_map[0].size(), ' ')); 

        for (size_t i = 0; i < obstacles.size(); ++i) {
            Point obs = obstacles[i];
            int obs_width = obstacle_sizes[i][0];
            int obs_height = obstacle_sizes[i][1];

            for (size_t x = static_cast<size_t>(obs.y); x < static_cast<size_t>(obs.y) + static_cast<size_t>(obs_height) && x < display_map.size(); ++x) {
                for (size_t y = static_cast<size_t>(obs.x); y < static_cast<size_t>(obs.x) + static_cast<size_t>(obs_width) && y < display_map[0].size(); ++y) {
                    display_map[x][y] = '#'; 
                }
            }
        }

        std::vector<Point> new_turn_points;
        for (size_t i = 1; i < path.size() - 1; ++i) {
            if (is_turn(path[i - 1], path[i], path[i + 1])) {
                new_turn_points.push_back(path[i]);
            }
        }

        setPoint.resize(2 + new_turn_points.size(), std::vector<int>(2));
        setPoint[0][0] = start.x; setPoint[0][1] = start.y;
        setPoint[1][0] = goal.x; setPoint[1][1] = goal.y;
        for (size_t i = 0; i < new_turn_points.size(); ++i) {
            setPoint[2 + i][0] = new_turn_points[i].x;
            setPoint[2 + i][1] = new_turn_points[i].y;
        }

        for (const auto& tp : new_turn_points) {
            display_map[tp.y][tp.x] = '+'; // 회전 지점 표시
        }

        display_map[start.y][start.x] = '*'; // 시작점 표시
        display_map[goal.y][goal.x] = '*'; // 목표점 표시

        size_t width = display_map[0].size();
        size_t height = display_map.size();

        // 상단의 @ 출력
        for (size_t i = 0; i < width + 2; ++i) {
            std::cout << "@ ";
        }
        std::cout << std::endl;

        for (size_t i = 0; i < height; ++i) {
            std::cout << "@ "; // 왼쪽 테두리(행의 시작)
            for (size_t j = 0; j < width; ++j) {
                if (display_map[i][j] == '#') { // 장애물 파란색
                    std::cout << BLUE << display_map[i][j] << RESET << ' '; 
                } else if (display_map[i][j] == '+') { // 회전 지점 노란색
                    std::cout << YELLOW << display_map[i][j] << RESET << ' '; 
                 // 시작점 발간색
                } else if (i == static_cast<size_t>(start.y) && j == static_cast<size_t>(start.x)) {
                    std::cout << RED << display_map[i][j] << RESET << ' '; 
                 // 목표점 초록색
                } else if (i == static_cast<size_t>(goal.y) && j == static_cast<size_t>(goal.x)) {
                    std::cout << GREEN << display_map[i][j] << RESET << ' '; 
                 // 빈 공간 기본 색상 공백
                } else {
                    std::cout << display_map[i][j] << ' '; 
                }
            }
             // 오른쪽 테두리(행의 끝)
            std::cout << "@ " << std::endl;
        }
        
        // 하단의 @ 출력 태두리
        for (size_t i = 0; i < width + 2; ++i) {
            std::cout << "@ ";
        }
        std::cout << std::endl;

        std::cout << "회전 지점 좌표들:" << std::endl;
        for (const auto& row : setPoint) {
            std::cout << "(" << row[0] << ", " << row[1] << ")" << std::endl;
        }
    }

    // 이동거리 계산 함수
    double calculate_path_distance(const std::vector<Point>& path) {
        double total_distance = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            double dx = path[i].x - path[i - 1].x;
            double dy = path[i].y - path[i - 1].y;
            // 유클리드 거리 계산
            total_distance += std::sqrt(dx * dx + dy * dy) * 10; 
        }
        return total_distance; 
    }
};

// 종료 시그널 처리 함수
void signalHandler(int /*signum*/) {
    cout << "Ctrl+C로 종료합니다." << endl;

    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0; 
    message.angular.z = 0.0; 

    auto node = rclcpp::Node::make_shared("stop_publisher");
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub->publish(message); 

    rclcpp::shutdown(); 
}

// 메인 함수
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); 

    auto node = std::make_shared<TurtlebotController>(); 

    std::thread control_thread(&TurtlebotController::robot_controller, node); 

    signal(SIGINT, signalHandler); 

    rclcpp::spin(node); 

    control_thread.join(); 

    return 0; 
}