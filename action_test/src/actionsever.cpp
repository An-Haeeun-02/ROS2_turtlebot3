#include <rclcpp/rclcpp.hpp> // ROS 2의 rclcpp 라이브러리 포함
#include <rclcpp_action/rclcpp_action.hpp> // ROS 2의 rclcpp_action 라이브러리 포함
#include "robot_action/action/test.hpp" // 사용자 정의 action 메시지 포함

// TestActionServer 클래스 정의, rclcpp::Node를 상속받음
class TestActionServer : public rclcpp::Node
{
public:
    // robot_action::action::Test 타입 정의
    using Test = robot_action::action::Test;
    // rclcpp_action::ServerGoalHandle<Test> 타입 정의
    using GoalHandleTest = rclcpp_action::ServerGoalHandle<Test>;

    // TestActionServer 생성자, "test_action_server" 노드 이름으로 초기화
    TestActionServer() : Node("test_action_server")
    {
        // "test" 이름의 action 서버 생성
        this->action_server_ = rclcpp_action::create_server<Test>(
            this, // 현재 노드
            "test", // 액션 이름
            std::bind(&TestActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2), // 목표 수신 콜백
            std::bind(&TestActionServer::handle_cancel, this, std::placeholders::_1), // 취소 요청 콜백
            std::bind(&TestActionServer::handle_accepted, this, std::placeholders::_1) // 목표 수락 콜백
        );
    }

private:
    // 액션 서버 공유 포인터
    rclcpp_action::Server<Test>::SharedPtr action_server_;

    // 목표 수신 콜백 함수
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID, std::shared_ptr<const Test::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request: %s", goal->message.c_str()); // 목표 메시지 로그 출력
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // 목표를 수락하고 실행
    }

    // 취소 요청 콜백 함수
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTest>)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request"); // 취소 요청 로그 출력
        return rclcpp_action::CancelResponse::ACCEPT; // 취소 요청 수락
    }

    // 목표 수락 콜백 함수
    void handle_accepted(const std::shared_ptr<GoalHandleTest> goal_handle)
    {
        std::thread{
            [this, goal_handle]() {
                const auto goal = goal_handle->get_goal(); // 목표 메시지 가져오기
                auto feedback = std::make_shared<Test::Feedback>(); // 피드백 메시지 생성
                auto result = std::make_shared<Test::Result>(); // 결과 메시지 생성

                feedback->feedback = "잘 받았습니다."; // 피드백 메시지 설정
                goal_handle->publish_feedback(feedback); // 피드백 메시지 발행

                // 일부 처리 시간을 시뮬레이션
                std::this_thread::sleep_for(std::chrono::seconds(2));

                result->response = "안녕하세요"; // 결과 메시지 설정
                goal_handle->succeed(result); // 목표 성공으로 완료
            }
        }.detach(); // 새 스레드에서 실행, 노드와의 독립적인 실행 보장
    }
};

// 메인 함수
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // rclcpp 초기화
    rclcpp::spin(std::make_shared<TestActionServer>()); // TestActionServer 노드 실행
    rclcpp::shutdown(); // rclcpp 종료
    return 0;
}
