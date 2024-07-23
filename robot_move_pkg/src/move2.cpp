#include <rclcpp/rclcpp.hpp>    // ROS2 C++ 클라이언트 라이브러리 포함
#include <geometry_msgs/msg/twist.hpp>  // geometry_msgs의 Twist 메시지 타입 포함
#include <termios.h>    // 터미널 제어를 위한 헤더 파일
#include <unistd.h>     // 유닉스 표준 함수들의 접근을 위한 헤더 파일
#include <stdio.h>      // 표준 입출력 함수를 위한 헤더 파일
#include <signal.h>     // 시그널 처리를 위한 헤더 파일

class TeleopTurtlebot : public rclcpp::Node
{
public:
  TeleopTurtlebot()
    : Node("teleop_turtlebot")  // 노드의 이름 설정
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);  // cmd_vel 토픽에 Twist 메시지를 퍼블리시하는 퍼블리셔 생성
    RCLCPP_INFO(this->get_logger(), "Press 'w' to move forward, 'a' to turn left, 'd' to turn right, 'x' to move backward, 's' to stop. Press Ctrl+C to quit.");  // 사용자에게 제어 방법을 안내하는 로그 출력
  }

  void keyLoop()
  {
    char c;   // 입력 받을 문자 변수 선언
    struct termios oldt, newt;  // 터미널 설정을 위한 구조체 변수 선언
    tcgetattr(STDIN_FILENO, &oldt);  // 현재 터미널 설정을 oldt에 저장
    newt = oldt;  // 새로운 터미널 설정을 oldt로 초기화
    newt.c_lflag &= ~(ICANON | ECHO);  // 캐노니컬 모드와 에코를 비활성화
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // 새로운 터미널 설정을 적용

    while (rclcpp::ok()) {  // ROS2가 실행 중인 동안 루프 실행
      c = getchar();  // 사용자의 키보드 입력을 읽음
      geometry_msgs::msg::Twist twist;  // Twist 메시지 객체 생성

      switch (c) {
        case 'w':  // 'w'를 입력받으면
          twist.linear.x = 0.2;  // 직선으로 전진하는 선형 속도 설정
          twist.angular.z = 0.0;  // 회전 속도는 0으로 설정
          break;
        case 'a':  // 'a'를 입력받으면
          twist.linear.x = 0.0;  // 직선으로 이동하지 않음
          twist.angular.z = 1.0;  // 좌회전을 위한 각속도 설정
          break;
        case 'd':  // 'd'를 입력받으면
          twist.linear.x = 0.0;  // 직선으로 이동하지 않음
          twist.angular.z = -1.0;  // 우회전을 위한 각속도 설정
          break;
        case 'x':  // 'x'를 입력받으면
          twist.linear.x = -0.2;  // 직선으로 후진하는 선형 속도 설정
          twist.angular.z = 0.0;  // 회전 속도는 0으로 설정
          break;
        case 's':  // 's'를 입력받으면
          twist.linear.x = 0.0;  // 직선으로 이동하지 않음
          twist.angular.z = 0.0;  // 회전 속도도 0으로 설정하여 정지
          break;
        default:  // 위의 case에 해당하지 않는 경우
          twist.linear.x = 0.0;  // 직선으로 이동하지 않음
          twist.angular.z = 0.0;  // 회전 속도도 0으로 설정하여 정지
          break;
      }

      publisher_->publish(twist);  // 설정된 Twist 메시지를 cmd_vel 토픽으로 퍼블리시
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 루프가 끝난 후에는 원래 터미널 설정으로 복구
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  // Twist 메시지를 퍼블리시하는 Publisher 포인터
};

// 프로그램 종료 시그널 핸들러 함수
void signalHandler(int signum)
{
  rclcpp::shutdown();  // ROS2 시스템을 종료

  // 터미널 설정을 원래 상태로 복구
  struct termios oldt;
  tcgetattr(STDIN_FILENO, &oldt);  // 현재 터미널 설정을 oldt에 저장
  oldt.c_lflag |= ICANON | ECHO;   // 캐노니컬 모드와 에코를 활성화
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 원래 터미널 설정으로 복구

  exit(signum);  // 프로그램 종료
}

// 메인 함수
int main(int argc, char **argv)
{
  signal(SIGINT, signalHandler);  // SIGINT 시그널을 받을 때 signalHandler 함수 실행
  rclcpp::init(argc, argv);  // ROS2 초기화
  auto node = std::make_shared<TeleopTurtlebot>();  // TeleopTurtlebot 노드 생성

  node->keyLoop();  // 키보드 입력 처리 루프 실행

  rclcpp::shutdown();  // ROS2 시스템 종료
  return 0;  // 프로그램 종료
}
