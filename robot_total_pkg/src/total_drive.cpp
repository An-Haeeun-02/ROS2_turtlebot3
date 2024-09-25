#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" // 퍼블리셔에서 사용하는 메시지 타입(Twist)
#include "geometry_msgs/msg/vector3.hpp" // 센서 데이터 구독에 사용하는 메시지 타입(Vector3)
#include <iostream>
#include <csignal>
#include <chrono>
#include <thread>
#include <map>

#include "sensor_msgs/msg/imu.hpp" // IMU 센서 데이터를 위한 메시지 타입
#include "nav_msgs/msg/odometry.hpp" // 로봇의 위치 데이터를 위한 메시지 타입
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // IMU 데이터를 처리하기 위한 변환 함수 포함
#include <atomic>
#include <cmath>

using namespace std;

// TurtlebotController 클래스 정의
class TurtlebotController : public rclcpp::Node {
private:
    // 퍼블리셔와 구독자 선언
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_; // cmd_vel 토픽에 메시지를 발행할 퍼블리셔
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_right_subscriber_; // 우측 센서 데이터 구독
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_left_subscriber_; // 좌측 센서 데이터 구독
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_front_subscriber_; // 전방 센서 데이터 구독
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_subscriber_; // 정면 센서 데이터 구독
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; // IMU 센서 데이터 구독
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // 오도메트리 데이터 구독

    // 센서 데이터 저장용 변수
    geometry_msgs::msg::Vector3 sensor_right_data_; // 우측 센서 데이터 저장
    geometry_msgs::msg::Vector3 sensor_left_data_; // 좌측 센서 데이터 저장
    geometry_msgs::msg::Vector3 sensor_front_data_; // 전방 센서 데이터 저장
    geometry_msgs::msg::Vector3 sensor_data_; // 정면 센서 데이터 저장

    // 상태 및 초기화 관련 변수
    bool obstacle_detected_; // 장애물 감지 플래그
    double previous_left_value_; // 이전 왼쪽 센서 값 저장
    double previous_right_value_; // 이전 오른쪽 센서 값 저장
    double initial_orientation_; // 초기 방향(로봇의 처음 방향)
    double current_orientation_; // 현재 방향(회전 값을 포함한 방향)
    
    double tfmini_range_;  // tfmini 거리 측정값 저장
    bool stair_detected_;  // 계단 감지 여부 플래그
    
    bool is_initialized_ = false; // 초기화 여부 체크 플래그
    bool stop_requested_ = false; // 중지 요청 플래그

    std::map<int, double> sensor_thresholds_; // 각도별 센서값 임계값

    // IMU 콜백 함수: 로봇의 현재 방향을 업데이트
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
        tf2::Quaternion quat; // 쿼터니언 데이터 받기
        tf2::fromMsg(msg->orientation, quat); // IMU 메시지로부터 쿼터니언으로 변환
        double roll, pitch, yaw; // 회전 각도(롤, 피치, 요)
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 쿼터니언을 롤, 피치, 요 각도로 변환

        current_orientation_ = yaw; // 현재 방향 업데이트

        if (!is_initialized_) {
          initial_orientation_ = yaw;  // 초기 방향 설정
          is_initialized_ = true; // 초기화 완료 플래그 설정
        }
    }

    // 로봇 제어 루프: 각도를 조정하여 로봇을 직진시킴
    void controlLoop(){
        double angle_diff = current_orientation_ - initial_orientation_; // 현재 각도와 초기 각도의 차이 계산
        cout << "직진 각도를 수정합니다." << endl; 

        // 각도 차이를 [-π, π] 범위로 정규화
        while (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI)
            angle_diff += 2 * M_PI;

        auto cmd_msg = geometry_msgs::msg::Twist(); // Twist 메시지 생성
        cmd_msg.linear.x = 0.2;  // 기본 직진 속도 설정

        // 각도 차이가 0.05 라디안 이상일 때 보정 회전
        if (std::fabs(angle_diff) > 0.05) {
            cmd_msg.angular.z = -angle_diff * 1.0;  // 각도 차이에 비례하여 회전 속도 설정
            cmd_msg.linear.x = 0.1;  // 회전 중일 때는 직진 속도 줄임
        } else {
            cmd_msg.angular.z = 0.0;  // 각도 차이가 작으면 회전하지 않음
        }

        // 퍼블리셔를 통해 명령 발행
        cmd_publisher_->publish(cmd_msg);
    }

public:
    // 생성자: 노드 초기화 및 센서, 퍼블리셔, 구독자 설정
    TurtlebotController() : Node("robot_cleaner"), 
        obstacle_detected_(false),
        stair_detected_(false), 
        previous_left_value_(0.0), 
        previous_right_value_(0.0) {

        sensor_thresholds_[0] = 25.0; // 정면 센서 임계값 설정
        sensor_thresholds_[30] = 30.0; // 우측 30도 센서 임계값 설정
        sensor_thresholds_[60] = 50.0; // 우측 60도 센서 임계값 설정
        sensor_thresholds_[300] = 50.0; // 좌측 300도 센서 임계값 설정
        sensor_thresholds_[330] = 30.0; // 좌측 330도 센서 임계값 설정

        // cmd_vel 토픽에 퍼블리셔 생성
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // 정면 센서 데이터 구독
        sensor_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("aeeun", 10,
            std::bind(&TurtlebotController::sensor_callback, this, std::placeholders::_1));
        // 우측 센서 데이터 구독
        sensor_right_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("hyewon", 10,
            std::bind(&TurtlebotController::sensor_right_callback, this, std::placeholders::_1));
        // 좌측 센서 데이터 구독
        sensor_left_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("dongwan", 10,
            std::bind(&TurtlebotController::sensor_left_callback, this, std::placeholders::_1));
        // IMU 센서 데이터 구독
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, 
            std::bind(&TurtlebotController::imuCallback, this, std::placeholders::_1));
        // 전방 센서 데이터 구독
        sensor_front_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("/haeeun", 10,
            std::bind(&TurtlebotController::sensor_front_callback, this, std::placeholders::_1));

        cout << "터틀봇 작동중..." << endl; // 터틀봇 초기화 완료 메시지 출력
    }

    // 정면 센서 콜백: 데이터를 저장하고, 임계값 및 장애물 확인
    void sensor_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        sensor_data_ = *msg; // 정면 센서 데이터 업데이트
        check_sensor_threshold(); // 센서 임계값 확인
        check_obstacle(); // 장애물 확인
    }

    // 우측 센서 콜백: 데이터를 저장
    void sensor_right_callback(const geometry_msgs::msg::Vector3::SharedPtr rmsg) {
        sensor_right_data_ = *rmsg; // 우측 센서 데이터 업데이트
    }

    // 좌측 센서 콜백: 데이터를 저장
    void sensor_left_callback(const geometry_msgs::msg::Vector3::SharedPtr lmsg) {
        sensor_left_data_ = *lmsg; // 좌측 센서 데이터 업데이트
    }

    // 전방 센서 콜백: 데이터를 저장하고 계단 여부 확인
    void sensor_front_callback(const geometry_msgs::msg::Vector3::SharedPtr fmsg) {
        sensor_front_data_ = *fmsg; // 전방 센서 데이터 업데이트
        check_stairs();  // 계단 여부 확인
    }

    // 계단 감지 함수: 계단 감지 시 로봇 정지
    void check_stairs() {
        if (sensor_front_data_.x > 15.0) {
            cout << "앞에 계단이 있습니다!" << endl; // 계단 감지 메시지 출력

            // 로봇을 멈추기 위한 Twist 메시지 발행
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            cmd_publisher_->publish(message);

            stair_detected_ = true; // 계단 감지 플래그 설정
        }
    }

    // 센서 임계값 확인 함수: 모든 센서 값이 임계값 이하일 경우 로봇 정지
    void check_sensor_threshold() {
        auto message = geometry_msgs::msg::Twist(); // Twist 메시지 객체 생성
        bool all_below_threshold = sensor_data_.x < sensor_thresholds_[0]; // 정면 센서 값이 임계값보다 작은지 확인

        bool right_sensor_below_threshold = sensor_right_data_.x < sensor_thresholds_[30] && sensor_right_data_.y < sensor_thresholds_[60]; // 우측 센서 값이 임계값 이하인지 확인
        bool left_sensor_below_threshold = sensor_left_data_.x < sensor_thresholds_[300] && sensor_left_data_.y < sensor_thresholds_[330]; // 좌측 센서 값이 임계값 이하인지 확인

        if (all_below_threshold && right_sensor_below_threshold && left_sensor_below_threshold) {
            cout << "모든 센서값이 설정값보다 작아 종료합니다." << endl; // 임계값 이하 메시지 출력

            // 로봇 정지 명령
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            cmd_publisher_->publish(message); // 명령 발행

            // 센서 데이터 로그 출력
            RCLCPP_INFO(this->get_logger(), "Sensor Data - 정면: %.2f, 우측: %.2f, 좌측: %.2f, 30도: %.2f, 330도: %.2f, 60도: %.2f, 300도: %.2f",
                        sensor_data_.x, sensor_data_.y, sensor_data_.z,
                        sensor_right_data_.x, sensor_right_data_.y,
                        sensor_left_data_.x, sensor_left_data_.y);

            rclcpp::shutdown(); // 노드 종료
        }
    }   

    // 장애물 확인 함수: 정면 센서 값이 특정 값 이하일 때 장애물 감지
    void check_obstacle() {
        obstacle_detected_ = sensor_data_.x < 30.0; // 장애물 감지 여부 확인
    }

    // 로봇 이동 함수: 장애물 및 계단을 감지하며 로봇을 이동시킴
    void move_robot() {
        while (rclcpp::ok()) { // 노드가 실행 중일 때 반복
            auto message = geometry_msgs::msg::Twist(); // Twist 메시지 객체 생성

            if (stair_detected_) { // 계단 감지 시 정지
                cout << "계단 감지로 인해 로봇 정지" << endl;
                break;
            }

            if (obstacle_detected_) { // 장애물 감지 시
                cout << "장애물 감지" << endl;
                message.linear.x = 0.0; // 전진 멈춤
                cmd_publisher_->publish(message); // 정지 명령 발행

                if (sensor_data_.y > sensor_data_.z) { // 좌측 센서 값이 더 크면 왼쪽으로 회전
                    left_course();
                } else { // 우측 센서 값이 더 크면 오른쪽으로 회전
                    right_course();
                }
                continue;
            } else { // 장애물 감지되지 않으면 계속 전진
                cout << "Moving forward" << endl;
                message.linear.x = 0.15; // 전진 속도 설정
                message.angular.z = 0.0;
                cmd_publisher_->publish(message); // 전진 명령 발행
                controlLoop(); // 각도 조정 루프 실행
            }

            this_thread::sleep_for(0.5s); // 0.5초 대기 후 반복
        }
    }

    // 오른쪽으로 회전하는 함수
    void right_course() {
        auto message = geometry_msgs::msg::Twist(); // Twist 메시지 객체 생성
        cout << "Turning right" << endl;

        right_turn(); // 오른쪽으로 회전
        cmd_publisher_->publish(message); // 회전 명령 발행
        this_thread::sleep_for(1s); // 회전 완료 대기

        // 회전 후 전진 명령 발행
        message.angular.z = 0.0;
        message.linear.x = 0.1;
        cmd_publisher_->publish(message);
        this_thread::sleep_for(0.5s); // 전진 시간 대기

        previous_left_value_ = sensor_data_.y; // 이전 왼쪽 센서 값 저장

        // 반복적으로 장애물 감지 및 센서값 증가 확인
        while (rclcpp::ok()) {
            if (obstacle_detected_) {
                cout << "정면 장애물 감지. right_course() 종료." << endl;
                return;
            }
            if (sensor_data_.y > previous_left_value_ + 10) { // 왼쪽 센서 값이 순간적으로 증가한 경우
                this_thread::sleep_for(0.5s);
                message.linear.x = 0.0;
                cmd_publisher_->publish(message);
                break;
            }
            this_thread::sleep_for(0.1s);
        }

        left_turn(); // 왼쪽으로 회전 후 명령 발행
        cmd_publisher_->publish(message);
        this_thread::sleep_for(1s); // 회전 완료 대기
    }

    // 왼쪽으로 회전하는 함수
    void left_course() {
        auto message = geometry_msgs::msg::Twist(); // Twist 메시지 객체 생성
        cout << "Turning left" << endl;

        left_turn(); // 왼쪽으로 회전
        cmd_publisher_->publish(message); // 회전 명령 발행
        this_thread::sleep_for(1s); // 회전 완료 대기

        // 회전 후 전진 명령 발행
        message.angular.z = 0.0;
        message.linear.x = 0.1;
        cmd_publisher_->publish(message);
        this_thread::sleep_for(0.5s); // 전진 시간 대기

        previous_right_value_ = sensor_data_.z; // 이전 오른쪽 센서 값 저장

        // 반복적으로 장애물 감지 및 센서값 증가 확인
        while (rclcpp::ok()) {
            if (obstacle_detected_) {
                cout << "정면 장애물 감지. left_course() 종료." << endl;
                return;
            }
            if (sensor_data_.z > previous_right_value_ + 10) { // 오른쪽 센서 값이 순간적으로 증가한 경우
                this_thread::sleep_for(0.5s);
                message.linear.x = 0.0;
                cmd_publisher_->publish(message);
                break;
            }
            this_thread::sleep_for(0.1s);
        }

        right_turn(); // 오른쪽으로 회전 후 명령 발행
        cmd_publisher_->publish(message);
        this_thread::sleep_for(1s); // 회전 완료 대기
    }

    // 오른쪽 회전 함수: 90도 회전
    void right_turn() {
        auto message = geometry_msgs::msg::Twist(); // Twist 메시지 객체 생성
        message.angular.z = -3.14 / 4.0; // 오른쪽으로 90도 회전
        cmd_publisher_->publish(message); // 회전 명령 발행
        std::this_thread::sleep_for(std::chrono::seconds(2)); // 회전 완료 대기
        message.angular.z = 0.0; // 회전 멈춤
        cmd_publisher_->publish(message); // 멈춤 명령 발행
    }

    // 왼쪽 회전 함수: 90도 회전
    void left_turn() {
        auto message = geometry_msgs::msg::Twist(); // Twist 메시지 객체 생성
        message.angular.z = 3.14 / 4.0; // 왼쪽으로 90도 회전
        cmd_publisher_->publish(message); // 회전 명령 발행
        std::this_thread::sleep_for(std::chrono::seconds(2)); // 회전 완료 대기
        message.angular.z = 0.0; // 회전 멈춤
        cmd_publisher_->publish(message); // 멈춤 명령 발행
    }
};

// Ctrl+C 시그널을 처리하는 핸들러 함수
void signalHandler(int /*signum*/) {
    cout << "Ctrl+C로 종료합니다." << endl; // 종료 메시지 출력

    // 로봇을 멈추기 위한 Twist 메시지 생성
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0; // 전진 멈춤
    message.angular.z = 0.0; // 회전 멈춤

    // 새로운 노드 생성하여 정지 명령 발행
    auto node = rclcpp::Node::make_shared("stop_publisher");
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub->publish(message); // 멈춤 명령 발행

    this_thread::sleep_for(1s); // 1초 대기 후 종료
    rclcpp::shutdown(); // 노드 종료
}

// 메인 함수
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); // ROS2 초기화
    auto node = make_shared<TurtlebotController>(); // TurtlebotController 노드 생성

    std::thread control_thread(&TurtlebotController::move_robot, node); // 로봇 이동 제어 스레드 실행

    signal(SIGINT, signalHandler); // Ctrl+C 시그널 핸들러 등록
    rclcpp::spin(node); // 노드 실행

    control_thread.join(); // 스레드 종료 대기

    return 0; // 프로그램 종료
}
