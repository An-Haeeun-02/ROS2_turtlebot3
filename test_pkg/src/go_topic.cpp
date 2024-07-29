#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>
#include <signal.h>
#include <iostream>
#include <csignal>

using namespace std;

// TurtlebotController 클래스 정의
class TurtlebotController : public rclcpp::Node {
private:
    // cmd_vel 토픽을 통해 Twist 메시지를 발행 (퍼블리셔)
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    // 센서 데이터를 수신 (서브스크라이버)
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_subscriber_;
    // sensor_data_ : 수신한 센서 데이터를 저장할 변수
    geometry_msgs::msg::Vector3 sensor_data_;
    // obstacle_detected_ : 장애물 감지 여부를 저장할 변수
    bool obstacle_detected_;
    // Odometry 관련 변수
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Point last_position_;
    double total_distance_;
    bool initialized_;

    void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 첫 번째 메시지 수신 시 초기화
        if (!initialized_) {
            last_position_ = msg->pose.pose.position;
            initialized_ = true;
            return;
        }
        // 현재 위치 추출
        auto current_position = msg->pose.pose.position;

        // 이전 위치와 현재 위치 간의 거리 계산
        double dx = current_position.x - last_position_.x;
        double dy = current_position.y - last_position_.y;

        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        // 총 이동 거리 업데이트
        total_distance_ += distance;

        if (total_distance_ >= 10.0) {  // 수정된 부분: 조건문 뒤에 세미콜론 제거
            RCLCPP_INFO(this->get_logger(), "Total Distance: %.2f meters", total_distance_);
            total_distance_ = 0.0;  // 이동 거리를 리셋 (옵션)
        }

        // 현재 위치를 이전 위치로 업데이트
        last_position_ = current_position;
    }

public:
    // 생성자: 노드 초기화 및 퍼블리셔, 서브스크립션 생성
    TurtlebotController() : Node("robot_cleaner"), obstacle_detected_(false), total_distance_(0.0), initialized_(false) {
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sensor_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("aeeun", 10,
            std::bind(&TurtlebotController::sensor_callback, this, std::placeholders::_1));
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->handle_odom(msg);
            });

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Total Distance: %.2f meters", total_distance_);
            }
        );

        cout << "터틀봇 작동중..." << endl;
    }

    // 센서 데이터를 수신했을 때 호출되는 콜백 함수
    void sensor_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        sensor_data_ = *msg;
        check_obstacle();  // 장애물 감지 여부
        move_robot();      // 로봇 제어
    }

    // 장애물 감지 여부를 판단하는 함수
    void check_obstacle() {
        obstacle_detected_ = sensor_data_.x < 30.0;  // x 값이 30.0보다 작으면 장애물 감지
    }

    // 로봇의 움직임을 제어하는 함수
    void move_robot() {
        auto message = geometry_msgs::msg::Twist();

        if (obstacle_detected_) {  // 장애물 감지
            cout << "장애물 감지" << endl;
            message.linear.x = 0.0;  // 전진을 멈춤

            if (sensor_data_.y > sensor_data_.z) {
                cout << "Turning left" << endl;
                message.angular.z = 2.5;  // 왼쪽 회전
            } else {
                cout << "Turning right" << endl;
                message.angular.z = -2.5;  // 오른쪽 회전
            }
        } else {
            // 장애물이 감지되지 않은 경우
            cout << "Moving forward" << endl;
            message.linear.x = 0.1;  // 전진
            message.angular.z = 0.0;  // 회전 없음
        }
        cmd_publisher_->publish(message);  // 명령 메시지 발행
    }
};

// Ctrl+C 시그널을 처리하는 핸들러 함수
void signalHandler(int /*signum*/) {
    cout << "Ctrl+C로 종료합니다." << endl;

    // 로봇을 정지시키기 위한 메시지 생성
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.0;

    // 로봇을 정지시키기 위해 cmd_vel 토픽에 메시지 발행
    auto node = rclcpp::Node::make_shared("stop_publisher");
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub->publish(message);

    rclcpp::shutdown();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<TurtlebotController>();
    // Ctrl+C 시그널 핸들러 등록
    signal(SIGINT, signalHandler);
    rclcpp::spin(node);

    return 0;
}
