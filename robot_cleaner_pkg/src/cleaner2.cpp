#include "rclcpp/rclcpp.hpp" // ROS 2 C++ 클라이언트 라이브러리 포함

#include "geometry_msgs/msg/twist.hpp" // Twist 메시지 타입 포함
#include "geometry_msgs/msg/vector3.hpp" // Vector3 메시지 타입 포함

#include <iostream> // 표준 입출력 사용

using namespace std; // std 네임스페이스 사용

// TurtlebotController 클래스 정의, rclcpp::Node를 상속받음
class TurtlebotController : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_; // Twist 메시지를 발행하는 Publisher
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sensor_subscriber_; // Vector3 메시지를 구독하는 Subscriber
    geometry_msgs::msg::Vector3 sensor_data_; // 센서 데이터를 저장하는 변수
    bool obstacle_detected_; // 장애물 감지 여부를 저장하는 변수
    bool rotating_; // 회전 중인지 여부를 저장하는 변수

public:
    // 생성자 정의
    TurtlebotController() : Node("turtlebot_controller"), obstacle_detected_(false), rotating_(false) {
        // cmd_vel 토픽에 Twist 메시지를 발행하는 Publisher 생성
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // aeeun 토픽에서 Vector3 메시지를 구독하는 Subscriber 생성
        sensor_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "aeeun", 10, std::bind(&TurtlebotController::sensor_callback, this, std::placeholders::_1));

        // 로그 출력
        RCLCPP_INFO(this->get_logger(), "Turtlebot is moving forward. Avoiding obstacles...");
    }

    // 센서 콜백 함수 정의
    void sensor_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        sensor_data_ = *msg; // 센서 데이터 업데이트
        check_obstacle(); // 장애물 감지 함수 호출
        
        if (!rotating_) { // 회전 중이 아닌 경우에만 move_robot 함수 호출
            move_robot(); // 센서 데이터가 업데이트될 때마다 move_robot 함수 호출
        }
    }

    // 장애물 감지 함수 정의
    void check_obstacle() {
        obstacle_detected_ = sensor_data_.x < 25.0; // x 축 데이터를 기준으로 장애물 여부 판단
    }

    // 로봇 움직임 함수 정의
    void move_robot() {
        auto message = geometry_msgs::msg::Twist(); // Twist 메시지 객체 생성

        if (obstacle_detected_) { // 장애물 감지 시
            RCLCPP_WARN(this->get_logger(), "Obstacle detected! Taking evasive action."); // 경고 로그 출력
            message.linear.x = 0.0; // 전진 정지

            if (sensor_data_.y > sensor_data_.z) { // 왼쪽으로 피하는 경우
                cout << "Turning left to avoid obstacle" << endl; // 왼쪽으로 회전하는 메시지 출력
                message.angular.z = 1.0; // 왼쪽으로 회전
            } else { // 오른쪽으로 피하는 경우
                cout << "Turning right to avoid obstacle" << endl; // 오른쪽으로 회전하는 메시지 출력
                message.angular.z = -1.0; // 오른쪽으로 회전
            }
            rotating_ = true; // 회전 시작
        } else { // 장애물이 없는 경우
            if (rotating_) { // 회전 중인 경우
                rotating_ = false; // 회전 종료
                cout << "Resuming forward movement" << endl; // 전진 재개 메시지 출력
            }
            
            cout << "Moving forward" << endl; // 전진하는 메시지 출력
            message.linear.x = 0.1; // 속도 설정
            message.angular.z = 0.0; // 각속도 초기화
        }

        cmd_publisher_->publish(message); // Twist 메시지 발행
    }
};

// 메인 함수 정의
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); // ROS 2 초기화
    auto node = make_shared<TurtlebotController>(); // TurtlebotController 객체 생성

    while (rclcpp::ok()) { // ROS가 실행 중인 동안 반복
        rclcpp::spin_some(node); // 노드의 콜백 함수 호출
    }
    rclcpp::shutdown(); // ROS 2 종료
    return 0; // 프로그램 종료
}
