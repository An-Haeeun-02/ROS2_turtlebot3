#include "rclcpp/rclcpp.hpp" 
#include "geometry_msgs/msg/twist.hpp"  // Twist 메시지 타입 포함
using namespace std;

class TestNode : public rclcpp::Node {
private:
    // 퍼블리셔 멤버 변수
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
public:
    // 생성자
    TestNode() : Node("rodot_move") {
        // 퍼블리셔 초기화, cmd_vel 토픽에 Twist 메시지 퍼블리시
        publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Move robot:\n"
                             "  'w': Move forward\n"
                             "  'a': Turn left\n"
                             "  'd': Turn right\n"
                             "  'x': Move backward\n"
                             "  's': Stop\n"
                             "Press Ctrl+C to quit.");
    }

    void move_robot(char command) {
        // Twist 메시지 객체 생성
        auto message = geometry_msgs::msg::Twist();
        // 입력된 명령에 따라 로봇의 움직임 설정
        switch (command) {
            case 'w':  //전진
                cout << "Moving forward" << endl;
                message.linear.x = 0.1;
                message.angular.z = 0.0;
                break;
            case 'x':  //후진
                cout << "Moving backward" << endl;
                message.linear.x = -0.1;
                message.angular.z = 0.0;
                break;
            case 'a':  //좌회전
                cout <<"Turn left" << endl;
                message.linear.x = 0.0;
                message.angular.z = 1.0;
                break;
            case 'd':  //우회전
                cout <<"Turn right" << endl;
                message.linear.x = 0.0;
                message.angular.z = -1.0;
                break;
            case 's':  //정지
                cout <<"Stop" << endl;
                message.linear.x = 0.0;
                message.angular.z = 0.0;
                break;
            default:  //정지
                RCLCPP_INFO(this->get_logger(), "Invalid command");
                message.linear.x = 0.0;
                message.angular.z = 0.0;
                break;
        }
        // Twist 메시지를 퍼블리시
        publisher->publish(message);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<TestNode>();

    while (rclcpp::ok()) {
        char command;  // 문자를 저장 변수
        cout << "> ";
        cin >> command;  //명령 입력
        node->move_robot(command);  // 입력된 명령에 따라 로봇 움직임 설정
		
        rclcpp::spin_some(node); //콜백함수 처리
    }
    rclcpp::shutdown();  // ROS2 시스템 종료
    return 0; 
}
