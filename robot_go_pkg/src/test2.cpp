#include <rclcpp/rclcpp.hpp>                  // ROS 2 C++ 클라이언트 라이브러리
#include <rclcpp_action/rclcpp_action.hpp>    // ROS 2 액션 라이브러리
#include <nav_msgs/msg/odometry.hpp>          // Odometry 메시지 타입
#include <geometry_msgs/msg/twist.hpp>        // Twist 메시지 타입
#include <geometry_msgs/msg/vector3.hpp>      // Vector3 메시지 타입
#include <geometry_msgs/msg/point.hpp>        // Point 메시지 타입
#include <iostream>                          // C++ 입출력 라이브러리
#include <csignal>                           // 시그널 처리 라이브러리
#include <chrono>                            // 시간 관련 라이브러리
#include "robot_action_pkg/action/move.hpp"   // 사용자 정의 액션 메시지 타입

using namespace std;

class TurtlebotController : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;   // Twist 메시지 발행을 위한 퍼블리셔
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_subscriber_; // Vector3 메시지 구독을 위한 서브스크라이버
    geometry_msgs::msg::Vector3 sensor_data_;   // 센서에서 수신된 데이터 저장 변수
    bool obstacle_detected_;                    // 장애물 감지 여부
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_; // Odometry 메시지 구독을 위한 서브스크라이버
    rclcpp_action::Server<robot_action_pkg::action::Move>::SharedPtr action_server_; // 액션 서버
    std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_action_pkg::action::Move>> current_goal_handle_; // 현재 목표 핸들
    geometry_msgs::msg::Point last_position_;  // 마지막 위치 저장 변수
    double total_distance_;                    // 총 이동 거리
    bool initialized_;                        // 초기화 상태
    double goal_distance_;                    // 목표 이동 거리

    // Odometry 메시지 콜백 함수
    void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 초기화가 되어 있지 않으면 현재 위치를 저장하고 초기화
        if (!initialized_) {
            last_position_ = msg->pose.pose.position;
            initialized_ = true;
            return;
        }

        // 현재 위치와 마지막 위치의 차이를 계산하여 이동 거리 계산
        auto current_position = msg->pose.pose.position;
        double dx = current_position.x - last_position_.x;
        double dy = current_position.y - last_position_.y;
        double dz = current_position.z - last_position_.z;
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        total_distance_ += distance;

        // 목표 핸들이 유효한 경우 피드백 발행
        if (current_goal_handle_) {
            auto feedback = std::make_shared<robot_action_pkg::action::Move::Feedback>();
            feedback->traveled_distances.push_back(total_distance_);
            current_goal_handle_->publish_feedback(feedback);
        }

        // 목표 거리 도달 시 결과를 발행하고 이동을 중지
        if (total_distance_ >= goal_distance_) {
            auto result = std::make_shared<robot_action_pkg::action::Move::Result>();
            result->success = true;
            if (current_goal_handle_) {
                current_goal_handle_->succeed(result);
            }
            total_distance_ = 0.0;
        }

        // 마지막 위치 업데이트
        last_position_ = current_position;
    }

    // 액션 목표 요청 처리
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const robot_action_pkg::action::Move::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with distance: %.2f meters", goal->distance);
        goal_distance_ = goal->distance; // 목표 거리 설정
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // 목표 수락 및 실행
    }

    // 목표 취소 요청 처리
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_action_pkg::action::Move>>)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT; // 취소 수락
    }

    // 목표 수락 후 처리
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_action_pkg::action::Move>> goal_handle)
    {
        current_goal_handle_ = goal_handle; // 현재 목표 핸들 저장
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.1; // 전진 속도 설정
        message.angular.z = 0.0; // 회전 속도 설정
        cmd_publisher_->publish(message); // 초기 명령어 발행

        // 이동 로직 실행 (1초 대기 시뮬레이션)
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

public:
    using Move = robot_action_pkg::action::Move;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

    TurtlebotController() 
        : Node("robot_cleaner"), obstacle_detected_(false), total_distance_{0.0}, initialized_{false}
    {
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // 명령어 발행을 위한 퍼블리셔 생성
        sensor_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("aeeun", 10,
            std::bind(&TurtlebotController::sensor_callback, this, std::placeholders::_1)); // 센서 데이터 구독

        RCLCPP_INFO(this->get_logger(), "Turtlebot running..."); // 로봇 실행 로그

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->handle_odom(msg); // Odometry 메시지 처리
            }
        );

        action_server_ = rclcpp_action::create_server<robot_action_pkg::action::Move>(
            this,
            "an",
            std::bind(&TurtlebotController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TurtlebotController::handle_cancel, this, std::placeholders::_1),
            std::bind(&TurtlebotController::handle_accepted, this, std::placeholders::_1)
        ); // 액션 서버 생성
    }

    // 센서 데이터 콜백 함수
    void sensor_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        sensor_data_ = *msg; // 센서 데이터 저장
        check_obstacle(); // 장애물 감지
        move_robot(); // 로봇 이동
    }

    // 장애물 감지
    void check_obstacle() {
        obstacle_detected_ = sensor_data_.x < 30.0; // 정면 거리 기준으로 장애물 감지
    }

    // 로봇 이동 로직
    void move_robot() {
        auto message = geometry_msgs::msg::Twist();

        if (obstacle_detected_) {
            RCLCPP_INFO(this->get_logger(), "Obstacle detected");
            message.linear.x = 0.0; // 장애물 발견 시 전진 속도 0으로 설정

            // 장애물 피하기 위해 회전
            if (sensor_data_.y > sensor_data_.z) {
                RCLCPP_INFO(this->get_logger(), "Turning left");
                message.angular.z = 2.5; // 왼쪽으로 회전
            } else {
                RCLCPP_INFO(this->get_logger(), "Turning right");
                message.angular.z = -2.5; // 오른쪽으로 회전
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Moving forward");
            message.linear.x = 0.1; // 장애물이 없으면 전진
            message.angular.z = 0.0; // 회전 속도 0으로 설정
        }
        cmd_publisher_->publish(message); // 명령어 발행
    }
};

// 시그널 핸들러 (Ctrl+C로 종료 시 호출됨)
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

// 메인 함수
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); // ROS 2 초기화
    auto node = std::make_shared<TurtlebotController>(); // TurtlebotController 노드 생성
    signal(SIGINT, signalHandler); // 시그널 핸들러 등록
    rclcpp::spin(node); // 노드 실행
    return 0; // 종료
}
