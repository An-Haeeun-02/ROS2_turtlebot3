#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // 퍼블리셔(보내는 쪽)
#include "geometry_msgs/msg/vector3.hpp" // 서브스크라이버(받는 쪽)
#include <iostream>
#include <csignal>
#include <chrono>
#include <thread>
#include <map>

using namespace std;

// TurtlebotController 클래스 정의
class TurtlebotController : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_right_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_left_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_subscriber_;
    geometry_msgs::msg::Vector3 sensor_right_data_;
    geometry_msgs::msg::Vector3 sensor_left_data_;
    geometry_msgs::msg::Vector3 sensor_data_;
    bool obstacle_detected_;
    double previous_left_value_;
    double previous_right_value_;

    
    // 각도별 센서값 임계값
    std::map<int, double> sensor_thresholds_;
    

public:
    // 생성자: 노드 초기화 및 퍼블리셔, 서브스크립션 생성
    TurtlebotController() : Node("robot_cleaner"), obstacle_detected_(false), previous_left_value_(0.0), previous_right_value_(0.0) {
        // 기본 임계값 설정 (각도별로 필요에 따라 변경 가능)
        sensor_thresholds_[0] = 25.0;
        sensor_thresholds_[30] = 30.0;
        sensor_thresholds_[60] = 50.0;
        sensor_thresholds_[300] = 50.0;
        sensor_thresholds_[330] = 30.0;

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sensor_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("aeeun", 10,
            std::bind(&TurtlebotController::sensor_callback, this, std::placeholders::_1));
        sensor_right_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("hyewon", 10,
            std::bind(&TurtlebotController::sensor_right_callback, this, std::placeholders::_1));
        sensor_left_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("dongwan", 10,
            std::bind(&TurtlebotController::sensor_left_callback, this, std::placeholders::_1));
        cout << "터틀봇 작동중..." << endl;
    }

    // 센서 데이터를 수신했을 때 호출되는 콜백 함수
    void sensor_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        sensor_data_ = *msg;
        check_sensor_threshold(); 
        check_obstacle(); 
    }

    void sensor_right_callback(const geometry_msgs::msg::Vector3::SharedPtr rmsg) {
        sensor_right_data_ = *rmsg;
    }

    void sensor_left_callback(const geometry_msgs::msg::Vector3::SharedPtr lmsg) {
        sensor_left_data_ = *lmsg;
    }

    // 각도별 설정값에 따라 종료 조건을 확인하는 함수
    void check_sensor_threshold() {
        auto message = geometry_msgs::msg::Twist();
        bool all_below_threshold = 
            sensor_data_.x < sensor_thresholds_[0];   // aeeun.x (0도)

        bool right_sensor_below_threshold = 
            sensor_right_data_.x < sensor_thresholds_[30] && // hyewon.x (30도)
            sensor_right_data_.y < sensor_thresholds_[60];  // hyewon.y (60도)

        bool left_sensor_below_threshold = 
            sensor_left_data_.x < sensor_thresholds_[300] && // dongwan.x (300도)
            sensor_left_data_.y < sensor_thresholds_[330];  // dongwan.y (330도)

        if (all_below_threshold && right_sensor_below_threshold && left_sensor_below_threshold) {
            cout << "모든 센서값이 설정값보다 작아 종료합니다." << endl;
        // 로봇을 멈추기 위한 Twist 메시지 발행
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            cmd_publisher_->publish(message);

            RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f, 30도: %.2f, 330도: %.2f, 60도: %.2f, 300도: %.2f"
                                , sensor_data_.x, sensor_data_.y, sensor_data_.z
                                , sensor_right_data_.x, sensor_right_data_.y
                                , sensor_left_data_.x, sensor_left_data_.y);

            rclcpp::shutdown();
        }
    }   

    // 장애물 감지 여부를 판단하는 함수
    void check_obstacle() {
        obstacle_detected_ = sensor_data_.x < 30.0; // x 값이 30.0보다 작으면 장애물 감지
    }

    // 로봇의 움직임을 제어하는 함수
    void move_robot() {
        while (rclcpp::ok()) {
            auto message = geometry_msgs::msg::Twist();

            if (obstacle_detected_) {
                cout << "장애물 감지" << endl;
                RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z);
                message.linear.x = 0.0; // 전진을 멈춤
                cmd_publisher_->publish(message); // 명령 메시지 발행
                if (sensor_data_.y > sensor_data_.z) {
                    left_course();
                } else {
                    right_course();
                }
                continue; // 반복문 시작
            } else {
                cout << "Moving forward" << endl;
                message.linear.x = 0.15; // 전진
                message.angular.z = 0.0; // 회전 없음
                cmd_publisher_->publish(message); // 명령 메시지 발행
            }

            // 멈춤 조건이 만족될 때까지 대기
            this_thread::sleep_for(0.5s); // 0.5초 대기
        }
    }

    void right_course() {
        auto message = geometry_msgs::msg::Twist();
        cout << "Turning right" << endl;
        RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z);

        // 1. 오른쪽으로 90도 회전
        right_turn();
        cmd_publisher_->publish(message);
        this_thread::sleep_for(1s); // 회전 완료 대기 (시간은 로봇의 회전 속도에 따라 조정 필요)

        // 2. 직진
        message.angular.z = 0.0; // 회전 없음
        message.linear.x = 0.1;
        cmd_publisher_->publish(message);
        this_thread::sleep_for(0.5s); // 직진 시간 조정 필요

        // 3. 왼쪽 값을 측정하며 이동
        previous_left_value_ = sensor_data_.y;
        while (rclcpp::ok()) {
            if (obstacle_detected_) {
                cout << "정면 장애물 감지. right_course() 종료." << endl;
                RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z);
                return; // 장애물 감지 시 함수 종료
            }
            if (sensor_data_.y > previous_left_value_ + 10) { // 왼쪽 값이 순간적으로 증가했을 때 멈춤
                this_thread::sleep_for(0.5s); // 직진 시간 조정 필
                message.linear.x = 0.0;
                cmd_publisher_->publish(message);
                break;
            }
            this_thread::sleep_for(0.1s); // 센서 체크 주기
        }

        // 4. 오른쪽으로 회전
        left_turn();
        cmd_publisher_->publish(message);
        this_thread::sleep_for(1s); // 회전 완료 대기 (시간은 로봇의 회전 속도에 따라 조정 필요)
    }

    void left_course() {
        auto message = geometry_msgs::msg::Twist();
        cout << "Turning left" << endl;
        RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z);

        // 1. 왼쪽으로 90도 회전
        left_turn();
        cmd_publisher_->publish(message);
        this_thread::sleep_for(1s); // 회전 완료 대기 (시간은 로봇의 회전 속도에 따라 조정 필요)

        // 2. 직진
        message.angular.z = 0.0; // 회전 없음
        message.linear.x = 0.1;
        cmd_publisher_->publish(message);
        this_thread::sleep_for(0.5s); // 직진 시간 조정 필요

        // 3. 오른쪽 값을 측정하며 이동
        previous_right_value_ = sensor_data_.z;
        while (rclcpp::ok()) {
            if (obstacle_detected_) {
                cout << "정면 장애물 감지. left_course() 종료." << endl;
                RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z);
                return; // 장애물 감지 시 함수 종료
            }
            if (sensor_data_.z > previous_right_value_ + 10) { // 오른쪽 값이 순간적으로 증가했을 때 멈춤
                this_thread::sleep_for(0.5s); // 직진 시간 조정 필요
                message.linear.x = 0.0;
                cmd_publisher_->publish(message);
                break;
            }
            this_thread::sleep_for(0.1s); // 센서 체크 주기
        }

        // 4. 오른쪽으로 회전
        right_turn();
        cmd_publisher_->publish(message);
        this_thread::sleep_for(1s); // 회전 완료 대기 (시간은 로봇의 회전 속도에 따라 조정 필요)
    }

    void right_turn() { // 오른쪽 회전
        RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z);
        auto message = geometry_msgs::msg::Twist();
        message.angular.z = -3.14 / 4.0; // 오른쪽으로 90도 회전
        cmd_publisher_->publish(message);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        message.angular.z = 0.0;
        cmd_publisher_->publish(message);
    }

    void left_turn() { // 왼쪽 회전
        RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z);
        auto message = geometry_msgs::msg::Twist();
        message.angular.z = 3.14 / 4.0; // 왼쪽으로 90도 회전
        cmd_publisher_->publish(message);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        message.angular.z = 0.0;
        cmd_publisher_->publish(message);
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

    // move_robot() 함수를 별도의 스레드에서 실행
    std::thread control_thread(&TurtlebotController::move_robot, node);

    // Ctrl+C 시그널 핸들러 등록
    signal(SIGINT, signalHandler);
    rclcpp::spin(node);

    // 스레드 종료 대기
    control_thread.join();

    return 0;
}