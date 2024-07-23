#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" //퍼블리셔(보내는 쪽)
#include "geometry_msgs/msg/vector3.hpp" //서브스크라이버(받는 쪽)
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

public:
    // 생성자: 노드 초기화 및 퍼블리셔, 서브스크립션 생성
    TurtlebotController() : Node("robot_cleaner"), obstacle_detected_(false) {//초기화
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);//cmd_vel Twist 퍼블리셔 
        sensor_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>("aeeun", 10, //aeeun Vector3 서브스크라이버
            std::bind(&TurtlebotController::sensor_callback, this, std::placeholders::_1));//콜백함수 포인터,this 로 맴버함수sensor_callback호출, 수신 메시지 인자로 받기
            cout << "터틀봇 작동중..." << endl;
    }

    // 센서 데이터를 수신했을 때 호출되는 콜백 함수
    void sensor_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        sensor_data_ = *msg;
        check_obstacle();//장애물 감지 여부
        move_robot(); // 로봇제어
    }

    // 장애물 감지 여부를 판단하는 함수
    void check_obstacle() {
        obstacle_detected_ = sensor_data_.x < 30.0; // x 값이 30.0보다 작으면 장애물 감지
    }

    // 로봇의 움직임을 제어하는 함수
    void move_robot() {
        auto message = geometry_msgs::msg::Twist();

        if (obstacle_detected_) {//장애물 감지
            cout << "장애물 감지" << endl;
            message.linear.x = 0.0; // 전진을 멈춤

            if (sensor_data_.y > sensor_data_.z) {
                cout << "Turning left" << endl;
                message.angular.z = 1.5708; //왼쪽 회전
            } else {
                cout << "Turning right" << endl;
                message.angular.z = -1.5708; //오른쪽 회전
            }
            cmd_publisher_->publish(message); // 명령 메시지 발행
            rclcpp::sleep_for(std::chrono::seconds(1));
        } else {
            // 장애물이 감지되지 않은 경우
            cout << "Moving forward" << endl;
            message.linear.x = 0.1; // 전진
            message.angular.z = 0.0; // 회전 없음
            cmd_publisher_->publish(message); // 명령 메시지 발행
        }
        
    }
};

// Ctrl+C 시그널을 처리하는 핸들러 함수
void signalHandler(int /*signum*/) {
    cout << "Ctrl+C로 종료합니다." << endl;

    // 로봇 정지 메시지 생성
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.0;

    // 로봇 정지 cmd_vel 토픽에 메시지 발행
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
    rclcpp::shutdown();
    return 0;
}
