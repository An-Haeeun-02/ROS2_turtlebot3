#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <algorithm>
#include <thread>
#include <csignal>

using namespace std;

#define RESET "\033[0m"
#define BLUE "\033[34m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"

class TurtlebotController : public rclcpp::Node {
public:
    TurtlebotController() : rclcpp::Node("turtlebot_controller") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&TurtlebotController::odom_callback, this, std::placeholders::_1));
    }

    void move_robot() {
        int width, height;
        std::cout << "그리드 너비를 입력하세요: ";
        std::cin >> width;
        std::cout << "그리드 높이를 입력하세요: ";
        std::cin >> height;

        grid_map = std::vector<std::vector<int>>(width, std::vector<int>(height, 0));

        std::cout << "시작 점 (x y)을 입력하세요: ";
        std::cin >> start.x >> start.y;
        std::cout << "목표 점 (x y)을 입력하세요: ";
        std::cin >> goal.x >> goal.y;

        int num_obstacles;
        std::cout << "장애물의 수를 입력하세요: ";
        std::cin >> num_obstacles;

        for (int i = 0; i < num_obstacles; ++i) {
            Point obstacle;
            int obs_width, obs_height;
            std::cout << "장애물 " << i + 1 << "의 위치 (x y)를 입력하세요: ";
            std::cin >> obstacle.x >> obstacle.y;
            std::cout << "장애물 " << i + 1 << "의 너비와 높이를 입력하세요: ";
            std::cin >> obs_width >> obs_height;

            obstacles.push_back(obstacle);
            obstacle_sizes.push_back({obs_width, obs_height});
        }

        set_obstacles();

        std::vector<Point> path = find_path();

        if (!path.empty()) {
            std::cout << "경로가 발견되었습니다:\n";
            print_grid_with_path(path);

            double total_distance = calculate_path_distance(path);
            std::cout << "총 경로 거리: " << total_distance << " cm" << std::endl;
            std::cout << "목표 지점에 도달했습니다!" << std::endl;
        } else {
            std::cout << "경로를 찾을 수 없습니다." << std::endl;
        }
    }

private:
    struct Point {
        int x, y;
        Point(int x = 0, int y = 0) : x(x), y(y) {}
        bool operator==(const Point& other) const {
            return x == other.x && y == other.y;
        }
    };

    enum Direction {
        UP, RIGHT, DOWN, LEFT
    };

    std::vector<std::vector<int>> grid_map;
    std::vector<std::vector<int>> obstacle_sizes;
    std::vector<Point> obstacles;
    Point start, goal;
    double distance_traveled = 0.0;
    std::vector<std::vector<int>> point;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    bool is_turn(const Point& prev, const Point& current, const Point& next) {
        return (prev.x != next.x && prev.y != next.y);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        static Point prev_position = start;
        double dx = msg->pose.pose.position.x - prev_position.x * 0.1;
        double dy = msg->pose.pose.position.y - prev_position.y * 0.1;
        distance_traveled += std::sqrt(dx * dx + dy * dy);
        prev_position = Point(msg->pose.pose.position.x / 0.1, msg->pose.pose.position.y / 0.1);
    }

    struct Node {
        Point point;
        Direction dir;
        int g, h;
        Node* parent;

        Node(Point p, Direction d, int g, int h, Node* parent = nullptr) 
            : point(p), dir(d), g(g), h(h), parent(parent) {}
        int f() const { return g + h; }

        bool operator>(const Node& other) const {
            return f() > other.f();
        }
    };

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

            if (current.point == goal) {
                std::vector<Point> path;
                while (current.parent) {
                    path.push_back(current.point);
                    if (current.parent && is_turn(current.parent->point, current.point, path.back())) {
                        turn_points.push_back(current.point);
                    }
                    current = *current.parent;
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());

                point.resize(2 + turn_points.size(), std::vector<int>(2));
                point[0][0] = start.x; point[0][1] = start.y;
                point[1][0] = goal.x; point[1][1] = goal.y;
                for (size_t i = 0; i < turn_points.size(); ++i) {
                    point[2 + i][0] = turn_points[i].x;
                    point[2 + i][1] = turn_points[i].y;
                }

                return path;
            }

            closed_list[current.point.x][current.point.y] = true;

            std::vector<std::pair<Point, Direction>> directions = {
                {{1, 0}, RIGHT},
                {{0, 1}, DOWN},
                {{-1, 0}, LEFT},
                {{0, -1}, UP}
            };

            for (const auto& [dir, direction] : directions) {
                Point next(current.point.x + dir.x, current.point.y + dir.y);

                if (next.x >= 0 && next.x < static_cast<int>(grid_map.size()) &&
                    next.y >= 0 && next.y < static_cast<int>(grid_map[0].size()) &&
                    grid_map[next.x][next.y] == 0 && !closed_list[next.x][next.y]) {

                    int rotation_cost = (current.dir == direction) ? 0 : 1;
                    open_list.push(Node(next, direction, current.g + 1 + rotation_cost, heuristic(next, goal), new Node(current)));
                }
            }
        }
        return {};
    }

    void set_obstacles() {
        for (size_t i = 0; i < obstacles.size(); ++i) {
            Point obs = obstacles[i];
            int obs_width = obstacle_sizes[i][0];
            int obs_height = obstacle_sizes[i][1];

            for (size_t x = static_cast<size_t>(obs.x); x < static_cast<size_t>(obs.x) + static_cast<size_t>(obs_width) && x < grid_map.size(); ++x) {
                for (size_t y = static_cast<size_t>(obs.y); y < static_cast<size_t>(obs.y) + static_cast<size_t>(obs_height) && y < grid_map[0].size(); ++y) {
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

            for (size_t x = static_cast<size_t>(obs.x); x < static_cast<size_t>(obs.x) + static_cast<size_t>(obs_width) && x < display_map.size(); ++x) {
                for (size_t y = static_cast<size_t>(obs.y); y < static_cast<size_t>(obs.y) + static_cast<size_t>(obs_height) && y < display_map[0].size(); ++y) {
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

        point.resize(2 + new_turn_points.size(), std::vector<int>(2));
        point[0][0] = start.x; point[0][1] = start.y;
        point[1][0] = goal.x; point[1][1] = goal.y;
        for (size_t i = 0; i < new_turn_points.size(); ++i) {
            point[2 + i][0] = new_turn_points[i].x;
            point[2 + i][1] = new_turn_points[i].y;
        }

        for (const auto& tp : new_turn_points) {
            display_map[tp.x][tp.y] = '+';
        }

        display_map[start.x][start.y] = '*';
        display_map[goal.x][goal.y] = '*';

        size_t width = display_map[0].size();
        size_t height = display_map.size();

        for (size_t i = 0; i < width + 2; ++i) {
            std::cout << "@ ";
        }
        std::cout << std::endl;

        for (size_t i = 0; i < height; ++i) {
            std::cout << "@ ";
            for (size_t j = 0; j < width; ++j) {
                if (display_map[i][j] == '#') {
                    std::cout << BLUE << display_map[i][j] << RESET << ' ';
                } else if (display_map[i][j] == '+') {
                    std::cout << YELLOW << display_map[i][j] << RESET << ' ';
                } else if (i == static_cast<size_t>(start.x) && j == static_cast<size_t>(start.y)) {
                    std::cout << RED << display_map[i][j] << RESET << ' ';
                } else if (i == static_cast<size_t>(goal.x) && j == static_cast<size_t>(goal.y)) {
                    std::cout << GREEN << display_map[i][j] << RESET << ' ';
                } else {
                    std::cout << display_map[i][j] << ' ';
                }
            }
            std::cout << "@ " << std::endl;
        }

        for (size_t i = 0; i < width + 2; ++i) {
            std::cout << "@ ";
        }
        std::cout << std::endl;

        std::cout << "회전 지점 좌표들:" << std::endl;
        for (const auto& row : point) {
            std::cout << "(" << row[0] << ", " << row[1] << ")" << std::endl;
        }
    }

    double calculate_path_distance(const std::vector<Point>& path) {
        double total_distance = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            double dx = path[i].x - path[i - 1].x;
            double dy = path[i].y - path[i - 1].y;
            total_distance += std::sqrt(dx * dx + dy * dy) * 10;
        }
        return total_distance;
    }
};

void signalHandler(int signum) {
    std::cout << "종료 시그널 수신: " << signum << std::endl;
    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TurtlebotController>();

    std::thread control_thread(&TurtlebotController::move_robot, node);

    signal(SIGINT, signalHandler);

    rclcpp::spin(node);

    control_thread.join();

    return 0;
}
