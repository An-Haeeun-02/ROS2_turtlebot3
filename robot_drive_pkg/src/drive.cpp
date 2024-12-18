#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // 퍼블리셔(보내는 쪽) 메시지 타입
#include "geometry_msgs/msg/vector3.hpp" // 서브스크라이버(받는 쪽) 메시지 타입
#include <iostream>
#include <csignal>
#include <chrono>
#include <thread>
#include <map>

using namespace std;

// TurtlebotController 클래스 정의
class TurtlebotController : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_; // cmd_vel 토픽에 메시지를 발행할 퍼블리셔
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_right_subscriber_; // 우측 센서 데이터 구독
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_left_subscriber_; // 좌측 센서 데이터 구독
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_subscriber_; // 정면 센서 데이터 구독
    geometry_msgs::msg::Vector3 sensor_right_data_; // 우측 센서 데이터 저장
    geometry_msgs::msg::Vector3 sensor_left_data_; // 좌측 센서 데이터 저장
    geometry_msgs::msg::Vector3 sensor_data_; // 정면 센서 데이터 저장
    bool obstacle_detected_; // 장애물 감지 플래그
    double previous_left_value_; // 이전 왼쪽 센서 값
    double previous_right_value_; // 이전 오른쪽 센서 값

    std::map<int, double> sensor_thresholds_; // 각도별 센서값 임계값

public:
    TurtlebotController() : Node("robot_cleaner"), obstacle_detected_(false), previous_left_value_(0.0), previous_right_value_(0.0) {
        // 기본 임계값 설정
        sensor_thresholds_[0] = 25.0;
        sensor_thresholds_[30] = 30.0;
        sensor_thresholds_[60] = 50.0;
        sensor_thresholds_[300] = 50.0;
        sensor_thresholds_[330] = 30.0;

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // cmd_vel 토픽에 퍼블리셔 생성
        sensor_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("aeeun", 10,
            std::bind(&TurtlebotController::sensor_callback, this, std::placeholders::_1)); // 정면 센서 데이터 구독
        sensor_right_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("hyewon", 10,
            std::bind(&TurtlebotController::sensor_right_callback, this, std::placeholders::_1)); // 우측 센서 데이터 구독
        sensor_left_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("dongwan", 10,
            std::bind(&TurtlebotController::sensor_left_callback, this, std::placeholders::_1)); // 좌측 센서 데이터 구독
        cout << "터틀봇 작동중..." << endl; // 초기화 완료 메시지 출력
    }

    void sensor_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        sensor_data_ = *msg; // 정면 센서 데이터 업데이트
        check_sensor_threshold(); // 센서 임계값 확인
        check_obstacle(); // 장애물 확인
    }

    void sensor_right_callback(const geometry_msgs::msg::Vector3::SharedPtr rmsg) {
        sensor_right_data_ = *rmsg; // 우측 센서 데이터 업데이트
    }

    void sensor_left_callback(const geometry_msgs::msg::Vector3::SharedPtr lmsg) {
        sensor_left_data_ = *lmsg; // 좌측 센서 데이터 업데이트
    }

    void check_sensor_threshold() {
        auto message = geometry_msgs::msg::Twist(); // 메시지 객체 생성
        bool all_below_threshold = 
            sensor_data_.x < sensor_thresholds_[0]; // 정면 센서 값이 임계값보다 작은지 확인

        bool right_sensor_below_threshold = 
            sensor_right_data_.x < sensor_thresholds_[30] && // 우측 x 값이 임계값보다 작은지 확인
            sensor_right_data_.y < sensor_thresholds_[60];  // 우측 y 값이 임계값보다 작은지 확인

        bool left_sensor_below_threshold = 
            sensor_left_data_.x < sensor_thresholds_[300] && // 좌측 x 값이 임계값보다 작은지 확인
            sensor_left_data_.y < sensor_thresholds_[330];  // 좌측 y 값이 임계값보다 작은지 확인

        if (all_below_threshold && right_sensor_below_threshold && left_sensor_below_threshold) {
            cout << "모든 센서값이 설정값보다 작아 종료합니다." << endl; // 모든 센서값이 임계값보다 작은 경우 메시지 출력

            message.linear.x = 0.0; // 전진 멈춤
            message.angular.z = 0.0; // 회전 멈춤
            cmd_publisher_->publish(message); // cmd_vel 토픽에 멈춤 명령 발행

            RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f, 30도: %.2f, 330도: %.2f, 60도: %.2f, 300도: %.2f"
                                , sensor_data_.x, sensor_data_.y, sensor_data_.z
                                , sensor_right_data_.x, sensor_right_data_.y
                                , sensor_left_data_.x, sensor_left_data_.y); // 센서 데이터 로그 출력

            rclcpp::shutdown(); // ROS2 노드 종료
        }
    }   

    void check_obstacle() {
        obstacle_detected_ = sensor_data_.x < 30.0; // x 값이 30.0보다 작으면 장애물 감지
    }

    void move_robot() {
        while (rclcpp::ok()) { // ROS2가 정상 동작하는 동안 반복
            auto message = geometry_msgs::msg::Twist(); // 메시지 객체 생성

            if (obstacle_detected_) { // 장애물 감지 시
                cout << "장애물 감지" << endl; // 장애물 감지 메시지 출력
                RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z); // 센서 데이터 로그 출력
                message.linear.x = 0.0; // 전진 멈춤
                cmd_publisher_->publish(message); // cmd_vel 토픽에 멈춤 명령 발행
                if (sensor_data_.y > sensor_data_.z) { // 좌측 센서 값이 우측 센서 값보다 클 때
                    left_course(); // 왼쪽으로 회전
                } else { // 우측 센서 값이 좌측 센서 값보다 클 때
                    right_course(); // 오른쪽으로 회전
                }
                continue; // 다음 루프 반복
            } else { // 장애물 감지 안 됨
                cout << "Moving forward" << endl; // 전진 메시지 출력
                message.linear.x = 0.15; // 전진 속도 설정
                message.angular.z = 0.0; // 회전 없음
                cmd_publisher_->publish(message); // cmd_vel 토픽에 전진 명령 발행
            }

            this_thread::sleep_for(0.5s); // 0.5초 대기
        }
    }

    void right_course() {
        auto message = geometry_msgs::msg::Twist(); // 메시지 객체 생성
        cout << "Turning right" << endl; // 오른쪽 회전 메시지 출력
        RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z); // 센서 데이터 로그 출력

        right_turn(); // 오른쪽으로 회전
        cmd_publisher_->publish(message); // 회전 명령 발행
        this_thread::sleep_for(1s); // 회전 완료 대기

        message.angular.z = 0.0; // 회전 멈춤
        message.linear.x = 0.1; // 직진 속도 설정
        cmd_publisher_->publish(message); // 직진 명령 발행
        this_thread::sleep_for(0.5s); // 직진 시간 대기

        previous_left_value_ = sensor_data_.y; // 이전 왼쪽 센서 값 저장
        while (rclcpp::ok()) { // ROS2가 정상 동작하는 동안 반복
            if (obstacle_detected_) { // 장애물 감지 시
                cout << "정면 장애물 감지. right_course() 종료." << endl; // 장애물 감지 메시지 출력
                RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z); // 센서 데이터 로그 출력
                return; // 함수 종료
            }
            if (sensor_data_.y > previous_left_value_ + 10) { // 왼쪽 값이 순간적으로 증가했을 때
                this_thread::sleep_for(0.5s); // 직진 시간 대기
                message.linear.x = 0.0; // 전진 멈춤
                cmd_publisher_->publish(message); // 멈춤 명령 발행
                break; // 루프 종료
            }
            this_thread::sleep_for(0.1s); // 센서 체크 주기
        }

        left_turn(); // 왼쪽으로 회전
        cmd_publisher_->publish(message); // 회전 명령 발행
        this_thread::sleep_for(1s); // 회전 완료 대기
    }

    void left_course() {
        auto message = geometry_msgs::msg::Twist(); // 메시지 객체 생성
        cout << "Turning left" << endl; // 왼쪽 회전 메시지 출력
        RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z); // 센서 데이터 로그 출력

        left_turn(); // 왼쪽으로 회전
        cmd_publisher_->publish(message); // 회전 명령 발행
        this_thread::sleep_for(1s); // 회전 완료 대기

        message.angular.z = 0.0; // 회전 멈춤
        message.linear.x = 0.1; // 직진 속도 설정
        cmd_publisher_->publish(message); // 직진 명령 발행
        this_thread::sleep_for(0.5s); // 직진 시간 대기

        previous_right_value_ = sensor_data_.z; // 이전 오른쪽 센서 값 저장
        while (rclcpp::ok()) { // ROS2가 정상 동작하는 동안 반복
            if (obstacle_detected_) { // 장애물 감지 시
                cout << "정면 장애물 감지. left_course() 종료." << endl; // 장애물 감지 메시지 출력
                RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z); // 센서 데이터 로그 출력
                return; // 함수 종료
            }
            if (sensor_data_.z > previous_right_value_ + 10) { // 오른쪽 값이 순간적으로 증가했을 때
                this_thread::sleep_for(0.5s); // 직진 시간 대기
                message.linear.x = 0.0; // 전진 멈춤
                cmd_publisher_->publish(message); // 멈춤 명령 발행
                break; // 루프 종료
            }
            this_thread::sleep_for(0.1s); // 센서 체크 주기
        }

        right_turn(); // 오른쪽으로 회전
        cmd_publisher_->publish(message); // 회전 명령 발행
        this_thread::sleep_for(1s); // 회전 완료 대기
    }

    void right_turn() {
        RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z); // 센서 데이터 로그 출력
        auto message = geometry_msgs::msg::Twist(); // 메시지 객체 생성
        message.angular.z = -3.14 / 4.0; // 오른쪽으로 90도 회전 설정
        cmd_publisher_->publish(message); // 회전 명령 발행
        std::this_thread::sleep_for(std::chrono::seconds(2)); // 회전 완료 대기
        message.angular.z = 0.0; // 회전 멈춤
        cmd_publisher_->publish(message); // 멈춤 명령 발행
    }

    void left_turn() {
        RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z); // 센서 데이터 로그 출력
        auto message = geometry_msgs::msg::Twist(); // 메시지 객체 생성
        message.angular.z = 3.14 / 4.0; // 왼쪽으로 90도 회전 설정
        cmd_publisher_->publish(message); // 회전 명령 발행
        std::this_thread::sleep_for(std::chrono::seconds(2)); // 회전 완료 대기
        message.angular.z = 0.0; // 회전 멈춤
        cmd_publisher_->publish(message); // 멈춤 명령 발행
    }
};

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

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); // ROS2 초기화
    auto node = make_shared<TurtlebotController>(); // TurtlebotController 노드 생성

    std::thread control_thread(&TurtlebotController::move_robot, node); // move_robot() 함수를 별도의 스레드에서 실행

    signal(SIGINT, signalHandler); // Ctrl+C 시그널 핸들러 등록
    rclcpp::spin(node); // 노드 실행

    control_thread.join(); // 스레드 종료 대기

    return 0; // 프로그램 종료
}
