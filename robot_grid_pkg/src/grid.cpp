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
    // 생성자 정의: 노드 이름 설정 및 odom 토픽 구독
    TurtlebotController() : rclcpp::Node("turtlebot_controller") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&TurtlebotController::odom_callback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    // 로봇을 움직이는 함수
    void make_root() {
        int width, height;
        std::cout << "그리드 너비를 입력하세요: ";  // 사용자로부터 그리드 너비 입력 받음
        std::cin >> width;
        std::cout << "그리드 높이를 입력하세요: ";  // 사용자로부터 그리드 높이 입력 받음
        std::cin >> height;

        // 그리드 맵 초기화
        grid_map = std::vector<std::vector<int>>(width, std::vector<int>(height, 0));

        // 시작점 입력
        std::cout << "시작 점 (x y)을 입력하세요: ";
        std::cin >> start.x >> start.y;

        // 목표점 입력
        std::cout << "목표 점 (x y)을 입력하세요: ";
        std::cin >> goal.x >> goal.y;

        // 장애물의 수 입력
        int num_obstacles;
        std::cout << "장애물의 수를 입력하세요: ";
        std::cin >> num_obstacles;

        // 장애물 정보 입력 (위치와 크기)
        for (int i = 0; i < num_obstacles; ++i) {
            Point obstacle;
            int obs_width, obs_height;
            std::cout << "장애물 " << i + 1 << "의 위치 (x y)를 입력하세요: ";
            std::cin >> obstacle.x >> obstacle.y;
            std::cout << "장애물 " << i + 1 << "의 너비와 높이를 입력하세요: ";
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
    void move_robot(){
        int block;
        int cm = 10;
        cout << "로봇을 움직입니다." << endl;
        //시작지점과 첫 중간지점 사이에 도착 지점이 들어있기 때문에 그 부분은 따로 해결
        cout << "시작->1번째 지점" << endl;
        cout << setPoint[0][0] <<","<< setPoint[0][1]<<" -> " << setPoint[2][0] << ","<<setPoint[2][1] << endl; 
            if(setPoint[0][1] == setPoint[2][1]){ //y가 같을 때
                block = setPoint[2][0]-setPoint[0][0];
                go_straight(block, cm);
            }else if(setPoint[0][0] == setPoint[2][0]){ //x가 같을때
                if(setPoint[0][1] > setPoint[2][1]){
                    block = setPoint[0][1] - setPoint[2][1];
                    left_course(block, cm);//좌회전 후 직진
                }else if(setPoint[0][1] < setPoint[2][1]){
                    block = setPoint[2][1] - setPoint[0][1];
                    right_course(block, cm);//우회전 후 직진
                }
            }
        //두번째 중간 지점 부터
        for(int i=2;setPoint.size()-1>i;i++ ){
            cout << i-1<<"번째 지점 -> " << i << "번째 지점"<< endl;
            cout << setPoint[i][0] <<","<< setPoint[i][1]<<" -> " << setPoint[i+1][0] << ","<<setPoint[i+1][1] << endl;
            if(setPoint[i][1] == setPoint[i+1][1]){ //y가 같을 때
                if (setPoint[i+1][0]<setPoint[i][0]){
                    block = setPoint[i][0]-setPoint[i+1][0];
                }else{
                    block = setPoint[i+1][0]-setPoint[i][0];  
                }
                go_straight(block, cm);
            }else if(setPoint[i][0] == setPoint[i+1][0]){ //x가 같을때
                if(setPoint[i][1] > setPoint[i+1][1]){
                    block = setPoint[i][1] - setPoint[i+1][1];
                    left_course(block, cm);//좌회전 후 직진
                }else if(setPoint[i][1] < setPoint[i+1][1]){
                    block = setPoint[i+1][1] - setPoint[i][1];
                    right_course(block, cm);//우회전 후 직진
                }
            }
        }
        //마지막 중간지점 부터 도착지점은 따로
        cout << setPoint.size()-2 <<"번째 지점 -> " << "도착지"<< endl;
        cout << setPoint[setPoint.size()-1][0] <<","<< setPoint[setPoint.size()-1][1]<<" -> " << setPoint[1][0] << ","<<setPoint[1][1] << endl;
            if(setPoint[setPoint.size()-1][1] == setPoint[1][1]){ //y가 같을 때
                block = setPoint[1][0]-setPoint[setPoint.size()-1][0];
                go_straight(block, cm);
            }else if(setPoint[setPoint.size()-1][0] == setPoint[1][0]){ //x가 같을때
                if(setPoint[setPoint.size()-1][1] > setPoint[1][1]){
                    block = setPoint[setPoint.size()-1][1] - setPoint[1][1];
                    left_course(block, cm);//좌회전 후 직진
                }else if(setPoint[setPoint.size()-1][1] < setPoint[1][1]){
                    block = setPoint[1][1] - setPoint[setPoint.size()-1][1];
                    right_course(block, cm);//우회전 후 직진
                }
            }
        cout <<"도착했습니다."<< endl;
    }
    // go_straight 함수
    void go_straight(int block, double cm) {
        cout << "직진"  << endl;
        // 이동해야 하는 거리 = block * cm
        double target_distance = block * cm;
        RCLCPP_INFO(this->get_logger(), "이동해야할 거리= %.2f cm", target_distance);

        // 이동 거리 초기화
        distance_traveled = 0.0;

        // Twist 메시지 생성 (로봇 이동 시작)
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = 0.1;  // 직진 속도 설정 (예: 0.2 m/s)
        cmd_msg.angular.z = 0.0; // 회전 속도는 0

        // 이동 시작
        while (rclcpp::ok() && distance_traveled < target_distance) {
            RCLCPP_INFO(this->get_logger(), "이동해야할 거리= %.2f cm / 이동 한 거리 = %.2f / 남은거리 = %.2f", 
                                            target_distance, distance_traveled * 100.0, target_distance-distance_traveled);
            cmd_publisher_->publish(cmd_msg);  // 속도 명령을 퍼블리시
            this_thread::sleep_for(0.5s);
        }

        // 목표 거리에 도달하면 정지
        cmd_msg.linear.x = 0.0;  // 직진 속도를 0으로 설정 (정지)
        cmd_publisher_->publish(cmd_msg);  // 정지 명령 퍼블리시
    }
    void right_course(int block, int cm){
        auto message = geometry_msgs::msg::Twist(); // 메시지 객체 생성
        cout << "Turning right" << endl;
        right_turn();
        go_straight(block, cm);
        left_turn();
    }
    void left_course(int block, int cm){
        auto message = geometry_msgs::msg::Twist();
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

    // 왼쪽으로 90도 회전
    void left_turn() {
        auto message = geometry_msgs::msg::Twist();
        message.angular.z = 3.14 / 4.0; // 왼쪽으로 90도 회전
        cmd_publisher_->publish(message);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        message.angular.z = 0.0;
        cmd_publisher_->publish(message);
    }
    void robot_controller(){
        // 먼저 make_root 실행
        make_root();
    
        // make_root가 완료되면 move_robot 실행
        move_robot();
    }

private:
    // 좌표 정보를 저장하기 위한 구조체 정의
    struct Point {
        int x, y;
        Point(int x = 0, int y = 0) : x(x), y(y) {}  // 생성자 정의
        bool operator==(const Point& other) const {  // 좌표 비교 연산자 오버로딩
            return x == other.x && y == other.y;
        }
    };

    // 방향을 나타내는 열거형 정의
    enum Direction {
        UP, RIGHT, DOWN, LEFT
    };

    std::vector<std::vector<int>> grid_map; // 그리드 맵을 저장할 벡터
    std::vector<std::vector<int>> obstacle_sizes; // 장애물 크기를 저장할 벡터
    std::vector<Point> obstacles;  // 장애물 위치를 저장할 벡터
    Point start, goal;  // 시작점과 목표점
    double distance_traveled = 0.0;  // 누적 이동 거리
    std::vector<std::vector<int>> setPoint; // 회전 지점 저장 (point를 setPoint로 변경)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // odom 토픽 구독자
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    // 로봇이 회전했는지 판단하는 함수
    bool is_turn(const Point& prev, const Point& current, const Point& next) {
        return (prev.x != next.x && prev.y != next.y); // x축과 y축이 동시에 변화하면 회전한 것
    }

    // odom 콜백 함수: 로봇의 위치를 업데이트하고 이동 거리를 계산
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        static double prev_x = 0.0, prev_y = 0.0;
        
        // 현재 위치 가져오기
        double current_x = msg->pose.pose.position.x * 100.0;
        double current_y = msg->pose.pose.position.y* 100.0;
        
        // 이전 위치와의 거리 계산
        double dx = current_x - prev_x;
        double dy = current_y - prev_y;
        double delta_distance = std::sqrt(dx * dx + dy * dy);
        
        // 이동한 거리 누적
        distance_traveled += delta_distance;

        // 이전 위치 갱신
        prev_x = current_x;
        prev_y = current_y;
    }


    // 경로 탐색에 사용될 노드 구조체 정의
    struct Node {
        Point point;  // 좌표
        Direction dir;  // 방향
        int g, h;  // g: 시작점부터의 비용, h: 휴리스틱 비용
        Node* parent;  // 부모 노드 포인터

        Node(Point p, Direction d, int g, int h, Node* parent = nullptr) 
            : point(p), dir(d), g(g), h(h), parent(parent) {}  // 생성자
        int f() const { return g + h; }  // 총 비용 계산

        // 우선순위 큐에서 사용할 비교 연산자 오버로딩
        bool operator>(const Node& other) const {
            return f() > other.f();
        }
    };

    // 휴리스틱 함수: 맨해튼 거리를 계산하여 추정 비용 반환
    int heuristic(const Point& a, const Point& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }

    // A* 알고리즘을 사용하여 경로를 찾는 함수
    std::vector<Point> find_path() {
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list; // 우선순위 큐
        std::vector<std::vector<bool>> closed_list(grid_map.size(), std::vector<bool>(grid_map[0].size(), false)); // 닫힌 목록

        open_list.push(Node(start, RIGHT, 0, heuristic(start, goal))); // 시작 노드 추가

        std::vector<Point> turn_points; // 회전 지점 목록

        while (!open_list.empty()) {  // 큐가 비어있지 않을 때까지 반복
            Node current = open_list.top();  // 가장 비용이 적은 노드를 가져옴
            open_list.pop();

            if (current.point == goal) {  // 목표 지점에 도달하면
                std::vector<Point> path;
                while (current.parent) {  // 경로를 추적하여 저장
                    path.push_back(current.point);
                    if (current.parent && is_turn(current.parent->point, current.point, path.back())) {
                        turn_points.push_back(current.point); // 회전 지점 저장
                    }
                    current = *current.parent;
                }
                path.push_back(start); // 시작점 추가
                std::reverse(path.begin(), path.end()); // 경로를 역순으로 변환

                // 회전 지점과 목표 지점을 저장
                setPoint.resize(2 + turn_points.size(), std::vector<int>(2));
                setPoint[0][0] = start.x; setPoint[0][1] = start.y;
                setPoint[1][0] = goal.x; setPoint[1][1] = goal.y;
                for (size_t i = 0; i < turn_points.size(); ++i) {
                    setPoint[2 + i][0] = turn_points[i].x;
                    setPoint[2 + i][1] = turn_points[i].y;
                }

                return path; // 경로 반환
            }

            closed_list[current.point.x][current.point.y] = true; // 현재 노드 닫음

            // 방향 이동 목록
            std::vector<std::pair<Point, Direction>> directions = {
                {{1, 0}, RIGHT},
                {{0, 1}, DOWN},
                {{-1, 0}, LEFT},
                {{0, -1}, UP}
            };

            for (const auto& [dir, direction] : directions) { // 네 방향에 대해 반복
                Point next(current.point.x + dir.x, current.point.y + dir.y); // 다음 좌표 계산

                // 그리드 경계 안에 있고, 장애물이 없으며, 닫힌 목록에 없는 경우
                if (next.x >= 0 && next.x < static_cast<int>(grid_map.size()) &&
                    next.y >= 0 && next.y < static_cast<int>(grid_map[0].size()) &&
                    grid_map[next.x][next.y] == 0 && !closed_list[next.x][next.y]) {

                    int rotation_cost = (current.dir == direction) ? 0 : 1;  // 방향이 달라지면 회전 비용 추가
                    open_list.push(Node(next, direction, current.g + 1 + rotation_cost, heuristic(next, goal), new Node(current))); // 새로운 노드 추가
                }
            }
        }
        return {}; // 경로를 찾지 못하면 빈 벡터 반환
    }

    // 장애물을 그리드 맵에 설정하는 함수
    void set_obstacles() {
        for (size_t i = 0; i < obstacles.size(); ++i) {
            Point obs = obstacles[i]; // 장애물 좌표
            int obs_width = obstacle_sizes[i][0]; // 장애물 너비
            int obs_height = obstacle_sizes[i][1]; // 장애물 높이

            // 그리드 맵에 장애물 설정
            for (size_t x = static_cast<size_t>(obs.x); x < static_cast<size_t>(obs.x) + static_cast<size_t>(obs_width) && x < grid_map.size(); ++x) {
                for (size_t y = static_cast<size_t>(obs.y); y < static_cast<size_t>(obs.y) + static_cast<size_t>(obs_height) && y < grid_map[0].size(); ++y) {
                    grid_map[x][y] = 1; // 장애물 위치를 1로 설정
                }
            }
        }
    }

    // 경로와 장애물을 포함한 그리드 맵을 출력하는 함수
    void print_grid_with_path(const std::vector<Point>& path) {
        std::vector<std::vector<char>> display_map(grid_map.size(), std::vector<char>(grid_map[0].size(), ' ')); // 시각화를 위한 맵 생성

        // 장애물 표시
        for (size_t i = 0; i < obstacles.size(); ++i) {
            Point obs = obstacles[i];
            int obs_width = obstacle_sizes[i][0];
            int obs_height = obstacle_sizes[i][1];

            for (size_t x = static_cast<size_t>(obs.x); x < static_cast<size_t>(obs.x) + static_cast<size_t>(obs_width) && x < display_map.size(); ++x) {
                for (size_t y = static_cast<size_t>(obs.y); y < static_cast<size_t>(obs.y) + static_cast<size_t>(obs_height) && y < display_map[0].size(); ++y) {
                    display_map[x][y] = '#'; // 장애물을 #로 표시
                }
            }
        }

        // 회전 지점 추가
        std::vector<Point> new_turn_points;
        for (size_t i = 1; i < path.size() - 1; ++i) {
            if (is_turn(path[i - 1], path[i], path[i + 1])) {
                new_turn_points.push_back(path[i]);
            }
        }

        // 회전 지점 저장
        setPoint.resize(2 + new_turn_points.size(), std::vector<int>(2));
        setPoint[0][0] = start.x; setPoint[0][1] = start.y;
        setPoint[1][0] = goal.x; setPoint[1][1] = goal.y;
        for (size_t i = 0; i < new_turn_points.size(); ++i) {
            setPoint[2 + i][0] = new_turn_points[i].x;
            setPoint[2 + i][1] = new_turn_points[i].y;
        }

        // 회전 지점을 +로 표시
        for (const auto& tp : new_turn_points) {
            display_map[tp.x][tp.y] = '+';
        }

        // 시작점과 목표점을 *로 표시
        display_map[start.x][start.y] = '*';
        display_map[goal.x][goal.y] = '*';

        size_t width = display_map[0].size();
        size_t height = display_map.size();

        // 그리드 테두리 출력
        for (size_t i = 0; i < width + 2; ++i) {
            std::cout << "@ ";
        }
        std::cout << std::endl;

        // 그리드 내용 출력
        for (size_t i = 0; i < height; ++i) {
            std::cout << "@ ";
            for (size_t j = 0; j < width; ++j) {
                if (display_map[i][j] == '#') {
                    std::cout << BLUE << display_map[i][j] << RESET << ' '; // 장애물 파란색으로 출력
                } else if (display_map[i][j] == '+') {
                    std::cout << YELLOW << display_map[i][j] << RESET << ' '; // 회전 지점 노란색으로 출력
                } else if (i == static_cast<size_t>(start.x) && j == static_cast<size_t>(start.y)) {
                    std::cout << RED << display_map[i][j] << RESET << ' '; // 시작점 빨간색으로 출력
                } else if (i == static_cast<size_t>(goal.x) && j == static_cast<size_t>(goal.y)) {
                    std::cout << GREEN << display_map[i][j] << RESET << ' '; // 목표점 초록색으로 출력
                } else {
                    std::cout << display_map[i][j] << ' '; // 빈 공간 출력
                }
            }
            std::cout << "@ " << std::endl;
        }

        // 그리드 테두리 출력
        for (size_t i = 0; i < width + 2; ++i) {
            std::cout << "@ ";
        }
        std::cout << std::endl;

        // 회전 지점 좌표 출력
        std::cout << "회전 지점 좌표들:" << std::endl;
        for (const auto& row : setPoint) {
            std::cout << "(" << row[0] << ", " << row[1] << ")" << std::endl;
        }
    }

    // 경로 거리를 계산하는 함수
    double calculate_path_distance(const std::vector<Point>& path) {
        double total_distance = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            double dx = path[i].x - path[i - 1].x;
            double dy = path[i].y - path[i - 1].y;
            total_distance += std::sqrt(dx * dx + dy * dy) * 10; // 경로 거리 계산
        }
        return total_distance; // 총 거리 반환
    }
};

// 종료 시그널 처리 함수
// Ctrl+C 시그널을 처리하는 핸들러 함수
void signalHandler(int /*signum*/) {
    cout << "Ctrl+C로 종료합니다." << endl; // 종료 메시지 출력

    auto message = geometry_msgs::msg::Twist(); // 메시지 객체 생성
    message.linear.x = 0.0; // 전진 멈춤
    message.angular.z = 0.0; // 회전 멈춤

    auto node = rclcpp::Node::make_shared("stop_publisher"); // 새로운 노드 생성
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // cmd_vel 토픽에 퍼블리셔 생성
    pub->publish(message); // 멈춤 명령 발행

    rclcpp::shutdown(); // ROS2 노드 종료
}

// 메인 함수
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); // ROS2 초기화

    auto node = std::make_shared<TurtlebotController>(); // TurtlebotController 노드 생성

    std::thread control_thread(&TurtlebotController::robot_controller, node); // 로봇 제어 스레드 실행

    signal(SIGINT, signalHandler); // Ctrl+C 신호 처리

    rclcpp::spin(node); // ROS2 노드 실행 (콜백 함수 대기)

    control_thread.join(); // 제어 스레드 종료 대기

    return 0; // 프로그램 종료
}
