#include <rclcpp/rclcpp.hpp>                   
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp> 
#include <geometry_msgs/msg/twist.hpp> 
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <thread>        
#include "robot_action/action/move.hpp" 
#include <iostream>                              
using namespace std;

class TurtlebotController : public rclcpp::Node {
public:
    using Move = robot_action::action::Move;     // Move 액션 메시지 타입을 정의(별칭)
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>; // Move 액션의 GoalHandle 타입을 정의

    TurtlebotController()
        : Node("robotController"),             // 노드 이름을 "robotController"로 초기화
          obstacle_detected_(false),            // 장애물 감지 플래그를 초기화
          total_distance_(0.0),                 // 총 이동 거리를 초기화
          initialized_(false),                  // 초기화 플래그를 초기화
          goal_distance_(0.0)                   // 목표 거리를 초기화
    {
        // "cmd_vel" 토픽 퍼블리셔 생성
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // "aeeun"토픽 서브스크라이버 생성
        sensor_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("aeeun", 10,
            std::bind(&TurtlebotController::sensor_callback, this, std::placeholders::_1));

        cout << "Turtlebot이 실행 중입니다..." << endl; //실행시 출력

        // "/odom" 토픽 서브스크라이버 생성
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&TurtlebotController::handle_odom, this, std::placeholders::_1)
        );

        // "an"이라는 Move 액션 서버를 생성
        action_server_ = rclcpp_action::create_server<Move>(
            this,
            "an",
            std::bind(&TurtlebotController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TurtlebotController::handle_cancel, this, std::placeholders::_1),
            std::bind(&TurtlebotController::handle_accepted, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_; // Twist 메시지를 퍼블리시할 퍼블리셔 변수
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_subscriber_; // Vector3 메시지를 수신할 구독자 변수
    geometry_msgs::msg::Vector3 sensor_data_; // 수신한 센서 데이터를 저장할 변수
    bool obstacle_detected_; // 장애물 감지 여부를 저장할 변수
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_; // Odometry 메시지를 수신할 구독자 변수
    rclcpp_action::Server<Move>::SharedPtr action_server_; // Move 액션 서버 변수
    std::shared_ptr<GoalHandleMove> current_goal_handle_; // 현재 목표 핸들 변수
    geometry_msgs::msg::Point last_position_; // 마지막 위치를 저장할 변수
    double total_distance_; // 총 이동 거리 변수
    bool initialized_; // 초기화 여부를 저장할 변수
    double goal_distance_; // 목표 거리 변수

    void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg) // Odometry 메시지 콜백 함수
    {
        if (!initialized_) { // 노드가 초기화되지 않은 경우
            last_position_ = msg->pose.pose.position; // 마지막 위치를 현재 위치로 설정
            initialized_ = true; // 초기화 완료
            return;
        }

        auto current_position = msg->pose.pose.position; // 현재 위치
        double dx = current_position.x - last_position_.x; // X축 방향 거리
        double dy = current_position.y - last_position_.y; // Y축 방향 거리
        double distance = std::sqrt(dx * dx + dy * dy); // 총 이동 거리
        total_distance_ += distance; // 총 이동 거리 업데이트

        if (current_goal_handle_) { // 현재 목표 핸들이 유효한 경우
            auto feedback = std::make_shared<Move::Feedback>(); // 피드백 메시지 생성
            feedback->traveled_distances.push_back(total_distance_); // 이동 거리 피드백에 추가
            current_goal_handle_->publish_feedback(feedback); // 피드백 메시지 발행
        }

        if (total_distance_ >= goal_distance_) { // 목표 거리를 초과한 경우
            auto result = std::make_shared<Move::Result>(); // 결과 메시지 생성
            result->success = true; // 결과를 성공으로 설정
            if (current_goal_handle_) {
                current_goal_handle_->succeed(result); // 목표를 성공으로 설정
                cout << "이동 완료" << endl;
                current_goal_handle_.reset(); // 목표 핸들 리셋
            }
            total_distance_ = 0.0; // 총 이동 거리 초기화
            goal_distance_ = 0.0; // 목표 거리 초기화
        }
        last_position_ = current_position; // 마지막 위치 업데이트
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Move::Goal> goal) // Goal 요청을 처리하는 함수
    {
        cout << "목표 거리: " << goal->distance << " 미터" << endl; // 목표 거리 표준 출력으로 출력
        goal_distance_ = goal->distance; // 목표 거리 설정
        total_distance_ = 0.0; // 총 이동 거리 초기화
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // 목표 수락 및 실행
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove>) // Goal 취소 요청을 처리하는 함수
    {
        cout << "취소 요청을 받았습니다" << endl; // 취소 요청 표준 출력으로 출력
        return rclcpp_action::CancelResponse::ACCEPT; // 취소 요청 수락
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) // 목표 수락 후 처리를 하는 함수
    {
        current_goal_handle_ = goal_handle; // 현재 목표 핸들 설정
        std::thread([this]() { // 새로운 스레드에서 작업 수행
            auto message = geometry_msgs::msg::Twist(); // Twist 메시지 생성
            while (rclcpp::ok() && current_goal_handle_) { // 노드가 정상 상태이며 현재 목표 핸들이 유효할 때 루프 실행
                if (obstacle_detected_) { // 장애물이 감지된 경우
                    message.linear.x = 0.0; // 이동 속도 0으로 설정
                    if (sensor_data_.y > sensor_data_.z) { // 센서 데이터에 따라 회전 방향 결정
                        message.angular.z = 2.5; // 오른쪽으로 회전
                    } else {
                        message.angular.z = -2.5; // 왼쪽으로 회전
                    }
                    cout << "장애물이 감지되었습니다: 정지하고 회전합니다" << endl; // 장애물 감지 표준 출력으로 출력
                } else {
                    message.linear.x = 0.2; // 이동 속도를 0.2로 설정
                    message.angular.z = 0.0; // 회전 속도 0으로 설정
                }
                cmd_publisher_->publish(message); // Twist 메시지 발행
                rclcpp::sleep_for(std::chrono::milliseconds(100)); // 100ms 대기
            }
            auto stop_message = geometry_msgs::msg::Twist(); // 이동 정지 메시지 생성
            stop_message.linear.x = 0.0; // 이동 속도 0으로 설정
            stop_message.angular.z = 0.0; // 회전 속도 0으로 설정
            cmd_publisher_->publish(stop_message); // 이동 정지 메시지 발행
        }).detach(); // 스레드를 분리하여 비동기적으로 실행
    }

    void sensor_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) { // 센서 데이터 수신 콜백 함수
        sensor_data_ = *msg; // 센서 데이터 저장
        check_obstacle(); // 장애물 여부 확인
    }//콜백 함수로 장애물 감지 여부와 센서 데이터 실시간으로 초기화

    void check_obstacle() { // 장애물 여부를 확인하는 함수
        obstacle_detected_ = sensor_data_.x < 30.0; // X값이 30보다 작은 경우 장애물 감지
    }
};

void signalHandler(int /*signum*/) { // Ctrl+C 신호를 처리하는 핸들러 함수
    cout << "Ctrl+C로 종료합니다." << endl; // 종료 메시지 표준 출력으로 출력

    auto message = geometry_msgs::msg::Twist(); // 이동 정지 메시지 생성
    message.linear.x = 0.0; // 이동 속도 0으로 설정
    message.angular.z = 0.0; // 회전 속도 0으로 설정

    auto node = rclcpp::Node::make_shared("stop_publisher"); // 새로운 노드 생성
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // "cmd_vel" 퍼블리셔 생성
    pub->publish(message); // 이동 정지 메시지 발행

    rclcpp::shutdown(); // ROS2 시스템 종료
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); // ROS2 시스템 초기화
    auto node = std::make_shared<TurtlebotController>(); // TurtlebotController 노드 생성
    signal(SIGINT, signalHandler); // SIGINT (Ctrl+C) 신호를 처리하도록 핸들러 설정
    rclcpp::spin(node); // 노드를 스핀하여 콜백을 처리

    return 0; // 프로그램 종료
}
