#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <csignal>
#include <thread>
#include "robot_action/action/move.hpp"

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

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp_action::Server<robot_action::action::Move>::SharedPtr action_server_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_action::action::Move>> current_goal_handle_;
    
    geometry_msgs::msg::Point last_position_;
    double total_distance_;
    bool initialized_;
    double goal_distance_;

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
        double dz = current_position.z - last_position_.z;

        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        // 총 이동 거리 업데이트
        total_distance_ += distance;

        // 피드백 전송
        auto feedback = std::make_shared<robot_action::action::Move::Feedback>();
        feedback->traveled_distances.push_back(total_distance_);
        if (current_goal_handle_) {
            current_goal_handle_->publish_feedback(feedback);
        }

        // 목표 거리 도달 시 이동 멈추기
        if (total_distance_ >= goal_distance_) {
            auto result = std::make_shared<robot_action::action::Move::Result>();
            result->success = true;
            current_goal_handle_->succeed(result);
            total_distance_ = 0.0; // 거리 초기화
        }

        last_position_ = current_position;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID,
        std::shared_ptr<const robot_action::action::Move::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with distance: %.2f meters", goal->distance);
        goal_distance_ = goal->distance; // 목표 거리 설정
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_action::action::Move>>)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_action::action::Move>> goal_handle)
    {
        current_goal_handle_ = goal_handle;
        std::thread{
            [this, goal_handle]() {
                auto message = geometry_msgs::msg::Twist();
                message.linear.x = 0.1; // 전진 속도 설정
                message.angular.z = 0.0; // 회전 속도 없음
                cmd_publisher_->publish(message);

                // 현재 목표 거리까지 이동할 때까지 대기
                rclcpp::sleep_for(std::chrono::seconds(1));
            }
        }.detach();
    }

public:
    // robot_action::action::Move 타입 정의
    using Move = robot_action::action::Move;
    // rclcpp_action::ServerGoalHandle<Move> 타입 정의
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

    // 생성자: 노드 초기화 및 퍼블리셔, 서브스크립션 생성
    TurtlebotController() 
        : Node("robot_cleaner"), obstacle_detected_(false), total_distance_{0.0}, initialized_{false}
    {
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sensor_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("aeeun", 10,
            std::bind(&TurtlebotController::sensor_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Turtlebot running...");

        // /odom 토픽을 구독합니다.
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->handle_odom(msg);
            }
        );

        // "an" 이름의 action 서버 생성
        action_server_ = rclcpp_action::create_server<robot_action::action::Move>(
            this,
            "an",
            std::bind(&TurtlebotController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TurtlebotController::handle_cancel, this, std::placeholders::_1),
            std::bind(&TurtlebotController::handle_accepted, this, std::placeholders::_1)
        );
    }

    // 센서 데이터를 수신했을 때 호출되는 콜백 함수
    void sensor_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        sensor_data_ = *msg;
        check_obstacle(); // 장애물 감지 여부
        move_robot(); // 로봇 제어
    }

    // 장애물 감지 여부를 판단하는 함수
    void check_obstacle() {
        obstacle_detected_ = sensor_data_.x < 30.0; // x 값이 30.0보다 작으면 장애물 감지
    }

    // 로봇의 움직임을 제어하는 함수
    void move_robot() {
        auto message = geometry_msgs::msg::Twist();

        if (obstacle_detected_) {
            RCLCPP_INFO(this->get_logger(), "Obstacle detected");
            message.linear.x = 0.0; // 전진을 멈춤

            if (sensor_data_.y > sensor_data_.z) {
                RCLCPP_INFO(this->get_logger(), "Turning left");
                message.angular.z = 2.5; // 왼쪽 회전
            } else {
                RCLCPP_INFO(this->get_logger(), "Turning right");
                message.angular.z = -2.5; // 오른쪽 회전
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Moving forward");
            message.linear.x = 0.1; // 전진
            message.angular.z = 0.0; // 회전 없음
        }
        cmd_publisher_->publish(message); // 명령 메시지 발행
    }
};

// Ctrl+C 시그널을 처리하는 핸들러 함수
void signalHandler(int /*signum*/) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down due to Ctrl+C");

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
