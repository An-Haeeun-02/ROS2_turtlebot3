#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // 퍼블리셔(보내는 쪽) 메시지 타입
#include "geometry_msgs/msg/vector3.hpp" // 서브스크라이버(받는 쪽) 메시지 타입
#include <iostream>
#include <csignal>
#include <chrono>
#include <thread>
#include <map>

#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <atomic>
#include <cmath>


using namespace std;

// TurtlebotController 클래스 정의
class TurtlebotController : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_; // cmd_vel 토픽에 메시지를 발행할 퍼블리셔
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_right_subscriber_; // 우측 센서 데이터 구독
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_left_subscriber_; // 좌측 센서 데이터 구독
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_front_subscriber_; // 정면 센서 데이터 구독
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_subscriber_; // 추가 정면 센서 데이터 구독
    geometry_msgs::msg::Vector3 sensor_right_data_; // 우측 센서 데이터 저장
    geometry_msgs::msg::Vector3 sensor_left_data_; // 좌측 센서 데이터 저장
    geometry_msgs::msg::Vector3 sensor_front_data_; // 정면 센서 데이터 저장
    geometry_msgs::msg::Vector3 sensor_data_; // 정면 센서 데이터 저장
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; // IMU 데이터 구독
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // Odometry 데이터 구독
    bool obstacle_detected_; // 장애물 감지 플래그
    double previous_left_value_; // 이전 왼쪽 센서 값 저장
    double previous_right_value_; // 이전 오른쪽 센서 값 저장
    double initial_orientation_; // 초기 방향 저장
    double current_orientation_; // 현재 방향 저장
    double tfmini_range_; // 센서 범위 저장
    bool stair_detected_; // 계단 감지 플래그
    bool is_initialized_ = false; // 초기화 상태 플래그
    bool stop_requested_ = false; // 정지 요청 플래그

    std::map<int, double> sensor_thresholds_; // 각도별 센서 임계값 설정

    // IMU 콜백 함수
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // IMU에서 현재 로봇의 방향을 가져옴
        tf2::Quaternion quat;
        tf2::fromMsg(msg->orientation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        current_orientation_ = yaw; // 현재 방향 업데이트

        if (!is_initialized_) {
            initial_orientation_ = yaw; // 초기 방향 설정
            is_initialized_ = true;    // 초기화 완료
        }
    }

    // 제어 루프 함수
    void controlLoop() {
        // 현재 각도와 초기 각도의 차이 계산
        double angle_diff = current_orientation_ - initial_orientation_;

        // 각도 차이를 [-π, π] 범위로 정규화
        while (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI)
            angle_diff += 2 * M_PI;

        // Twist 메시지 생성
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.2;  // 기본 직진 속도 설정

        // 각도 차이에 따라 회전 속도 조정
        if (std::fabs(angle_diff) > 0.05) { // 각도 차이가 0.05 라디안 이상일 때만 보정
            cout << "직진 각도를 수정합니다." << endl;
            message.angular.z = -angle_diff * 1.0;  // 각도 차이에 비례하여 회전 속도 설정
            message.linear.x = 0.1;  // 회전 중일 때는 직진 속도를 줄임
        } else {
            message.angular.z = 0.0;  // 각도 차이가 작으면 회전하지 않음
        }
        // 명령 퍼블리시
        cmd_publisher_->publish(message);
    }

public:
    // TurtlebotController 클래스 생성자
    TurtlebotController() : Node("robot_cleaner"),
        obstacle_detected_(false),
        stair_detected_(false),
        previous_left_value_(0.0),
        previous_right_value_(0.0) {
        // 기본 임계값 설정
        sensor_thresholds_[0] = 25.0;
        sensor_thresholds_[30] = 30.0;
        sensor_thresholds_[60] = 50.0;
        sensor_thresholds_[300] = 50.0;
        sensor_thresholds_[330] = 30.0;

        // 퍼블리셔 및 구독자 초기화
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sensor_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("aeeun", 10,
            std::bind(&TurtlebotController::sensor_callback, this, std::placeholders::_1));
        sensor_right_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("hyewon", 10,
            std::bind(&TurtlebotController::sensor_right_callback, this, std::placeholders::_1));
        sensor_left_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("dongwan", 10,
            std::bind(&TurtlebotController::sensor_left_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10,
            std::bind(&TurtlebotController::imuCallback, this, std::placeholders::_1));
        sensor_front_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("/haeeun", 10,
            std::bind(&TurtlebotController::sensor_front_callback, this, std::placeholders::_1));

        cout << "터틀봇 작동중..." << endl; // 초기화 완료 메시지 출력
    }

    // 정면 센서 콜백 함수
    void sensor_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        sensor_data_ = *msg; // 정면 센서 데이터 업데이트
        check_sensor_threshold(); // 센서 임계값 확인
        check_obstacle(); // 장애물 확인
    }

    // 우측 센서 콜백 함수
    void sensor_right_callback(const geometry_msgs::msg::Vector3::SharedPtr rmsg) {
        sensor_right_data_ = *rmsg; // 우측 센서 데이터 업데이트
    }

    // 좌측 센서 콜백 함수
    void sensor_left_callback(const geometry_msgs::msg::Vector3::SharedPtr lmsg) {
        sensor_left_data_ = *lmsg; // 좌측 센서 데이터 업데이트
    }

    // 정면 센서 콜백 함수
    void sensor_front_callback(const geometry_msgs::msg::Vector3::SharedPtr fmsg) {
        sensor_front_data_ = *fmsg; // 정면 센서 데이터 업데이트
        check_stairs();  // 계단 여부 확인
    }

    // 계단 감지 함수
    void check_stairs() {
        if (sensor_front_data_.x > 15.0) {
            cout << "앞에 계단이 있습니다!" << endl;

            // 로봇을 멈추기 위한 Twist 메시지 발행
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            cmd_publisher_->publish(message);
            stair_detected_ = true; // 계단 감지 상태 업데이트
        }
    }

    // 센서 임계값 확인 함수
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
            cout << "모든 센서값이 설정값보다 작아 종료합니다." << endl;

            // 로봇 멈춤 명령
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            cmd_publisher_->publish(message);

            // 센서 데이터 로그 출력
            RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f, 30도: %.2f, 330도: %.2f, 60도: %.2f, 300도: %.2f"
                        , sensor_data_.x, sensor_data_.y, sensor_data_.z
                        , sensor_right_data_.x, sensor_right_data_.y
                        , sensor_left_data_.x, sensor_left_data_.y);

            rclcpp::shutdown(); // ROS2 노드 종료
        }
    }

    // 장애물 감지 함수
    void check_obstacle() {
        obstacle_detected_ = sensor_data_.x < 30.0; // 정면 x 값이 30.0보다 작으면 장애물 감지
    }

    // 로봇 이동 함수
    void move_robot() {
        while (rclcpp::ok()) { // ROS2가 정상 동작하는 동안 반복
            auto message = geometry_msgs::msg::Twist(); // 메시지 객체 생성
            if (stair_detected_) {
                // 계단 감지 시 전진 중지
                cout << "계단 감지로 인해 로봇 정지" << endl;
                break;
            }

            if (obstacle_detected_) { // 장애물 감지 시
                cout << "장애물 감지" << endl;
                RCLCPP_INFO(this->get_logger(), "실제 센서 데이터 - 정면: %.2f, 우측 30도: %.2f, 우측 60도: %.2f, 좌측 300도: %.2f, 좌측 330도: %.2f",
                    sensor_front_data_.x, sensor_right_data_.x, sensor_right_data_.y, sensor_left_data_.x, sensor_left_data_.y);
                message.linear.x = 0.0; // 전진 멈춤
                message.angular.z = 0.0; // 회전 없음
                cmd_publisher_->publish(message);

                if (sensor_data_.y > sensor_data_.z) { // 좌측 센서 값이 우측 센서 값보다 크면
                    left_course(); // 왼쪽으로 회전
                } else { // 우측 센서 값이 좌측 센서 값보다 크면
                    right_course(); // 오른쪽으로 회전
                }
                continue; // 다음 루프 반복
            } else { // 장애물 감지 안 됨
                cout << "Moving forward" << endl;
                message.linear.x = 0.15; // 전진 속도 설정
                cmd_publisher_->publish(message); // cmd_vel 토픽에 전진 명령 발행
                controlLoop(); // 각도 보정
            }

            this_thread::sleep_for(0.5s); // 0.5초 대기
        }
    }

    // 오른쪽 회전 경로 설정 함수
    void right_course() {
        auto message = geometry_msgs::msg::Twist(); // 메시지 객체 생성
        cout << "Turning right" << endl;
        RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z);

        right_turn(); // 오른쪽 회전 명령
        cmd_publisher_->publish(message); // 회전 명령 발행
        this_thread::sleep_for(1s); // 회전 완료 대기

        message.angular.z = 0.0; // 회전 멈춤
        message.linear.x = 0.1; // 직진 속도 설정
        cmd_publisher_->publish(message);
        this_thread::sleep_for(0.5s);

        previous_left_value_ = sensor_data_.y; // 이전 왼쪽 센서 값 저장
        while (rclcpp::ok()) {
            if (obstacle_detected_) {
                cout << "정면 장애물 감지. right_course() 종료." << endl;
                RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z);
                return;
            }
            if (sensor_data_.y > previous_left_value_ + 10) {
                this_thread::sleep_for(0.5s); // 직진 시간 대기
                message.linear.x = 0.0;
                cmd_publisher_->publish(message); // 멈춤 명령
                break;
            }
            this_thread::sleep_for(0.1s); // 센서 체크 주기
        }

        left_turn(); // 왼쪽으로 회전
        cmd_publisher_->publish(message);
        this_thread::sleep_for(1s);
    }

    // 왼쪽 회전 경로 설정 함수
    void left_course() {
        auto message = geometry_msgs::msg::Twist();
        cout << "Turning left" << endl;
        RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z);

        left_turn(); // 왼쪽 회전 명령
        cmd_publisher_->publish(message);
        this_thread::sleep_for(1s);

        message.angular.z = 0.0; // 회전 멈춤
        message.linear.x = 0.1; // 직진 속도 설정
        cmd_publisher_->publish(message);
        this_thread::sleep_for(0.5s);

        previous_right_value_ = sensor_data_.z;
        while (rclcpp::ok()) {
            if (obstacle_detected_) {
                cout << "정면 장애물 감지. left_course() 종료." << endl;
                RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z);
                return;
            }
            if (sensor_data_.z > previous_right_value_ + 10) {
                this_thread::sleep_for(0.5s);
                message.linear.x = 0.0;
                cmd_publisher_->publish(message);
                break;
            }
            this_thread::sleep_for(0.1s);
        }

        right_turn(); // 오른쪽으로 회전
        cmd_publisher_->publish(message);
        this_thread::sleep_for(1s);
    }

    // 오른쪽으로 90도 회전
    void right_turn() {
        RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f", sensor_data_.x, sensor_data_.y, sensor_data_.z);
        auto message = geometry_msgs::msg::Twist();
        message.angular.z = -3.14 / 4.0; // 오른쪽으로 90도 회전
        cmd_publisher_->publish(message);
        std::this_thread::sleep_for(std::chrono::seconds(2)); // 회전 대기
        message.angular.z = 0.0; // 회전 멈춤
        cmd_publisher_->publish(message);
    }

    // 왼쪽으로 90도 회전
    void left_turn() {
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

    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0; // 전진 멈춤
    message.angular.z = 0.0; // 회전 멈춤

    auto node = rclcpp::Node::make_shared("stop_publisher"); // 새로운 노드 생성
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // 퍼블리셔 생성
    pub->publish(message); // 멈춤 명령 발행
    this_thread::sleep_for(1s);
    rclcpp::shutdown(); // ROS2 노드 종료
}

// 메인 함수
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); // ROS2 초기화
    auto node = make_shared<TurtlebotController>(); // TurtlebotController 노드 생성

    std::thread control_thread(&TurtlebotController::move_robot, node); // move_robot() 함수를 별도의 스레드에서 실행

    signal(SIGINT, signalHandler); // Ctrl+C 시그널 핸들러 등록
    rclcpp::spin(node); // 노드 실행

    control_thread.join(); // 스레드 종료 대기

    return 0; // 프로그램 종료
}
