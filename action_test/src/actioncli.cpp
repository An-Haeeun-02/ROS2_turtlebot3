#include <rclcpp/rclcpp.hpp> // ROS 2의 rclcpp 라이브러리 포함
#include <rclcpp_action/rclcpp_action.hpp> // ROS 2의 rclcpp_action 라이브러리 포함
#include "robot_action/action/test.hpp" // 사용자 정의 action 메시지 포함

// TestActionClient 클래스 정의, rclcpp::Node를 상속받음
class TestActionClient : public rclcpp::Node
{
public:
    // robot_action::action::Test 타입 정의
    using Test = robot_action::action::Test;
    // rclcpp_action::ClientGoalHandle<Test> 타입 정의
    using GoalHandleTest = rclcpp_action::ClientGoalHandle<Test>;

    // TestActionClient 생성자, "Test_action_client" 노드 이름으로 초기화
    TestActionClient() : Node("Test_action_client")
    {
        // "test" 이름의 action 클라이언트 생성
        this->client_ = rclcpp_action::create_client<Test>(
            this,
            "test"
        );

        // 목표 전송 함수 호출
        this->send_goal();
    }

private:
    // 클라이언트 공유 포인터
    rclcpp_action::Client<Test>::SharedPtr client_;

    // 목표를 전송하는 함수
    void send_goal()
    {
        // 액션 서버를 기다림, 5초 동안 대기
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting"); // 액션 서버가 사용 불가능할 때 오류 메시지 출력
            return;
        }

        // 목표 메시지 생성 및 설정
        auto goal_msg = Test::Goal();
        goal_msg.message = "안녕"; // 목표 메시지 설정

        // 목표 옵션 생성
        auto goal_options = rclcpp_action::Client<Test>::SendGoalOptions();
        
        // 목표 응답 콜백 정의
        goal_options.goal_response_callback = [](std::shared_ptr<GoalHandleTest> goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected"); // 목표가 거부되었을 때 오류 메시지 출력
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted"); // 목표가 수락되었을 때 정보 메시지 출력
        };

        // 피드백 콜백 정의
        goal_options.feedback_callback = [this](GoalHandleTest::SharedPtr, const std::shared_ptr<const Test::Feedback> feedback) {
            RCLCPP_INFO(this->get_logger(), "Received feedback: %s", feedback->feedback.c_str()); // 피드백 수신 시 정보 메시지 출력
        };

        // 결과 콜백 정의
        goal_options.result_callback = [this](const GoalHandleTest::WrappedResult &result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Result received: %s", result.result->response.c_str()); // 성공 시 결과 메시지 출력
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Result was aborted"); // 중단 시 오류 메시지 출력
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Result was cancelled"); // 취소 시 오류 메시지 출력
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Result code: UNKNOWN"); // 알 수 없는 결과 코드 시 오류 메시지 출력
                    break;
            }
        };

        // 비동기 목표 전송
        auto goal_handle_future = client_->async_send_goal(goal_msg, goal_options);

        // 목표 전송 완료를 기다리고 처리
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal"); // 목표 전송 실패 시 오류 메시지 출력
            return;
        }

        auto goal_handle = goal_handle_future.get(); // 목표 핸들 가져오기
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "No goal handle"); // 목표 핸들 없음 오류 메시지 출력
            return;
        }

        // 결과를 기다리거나 추가 작업을 수행할 수 있음
    }
};

// 메인 함수
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // rclcpp 초기화
    rclcpp::spin(std::make_shared<TestActionClient>()); // TestActionClient 노드 실행
    rclcpp::shutdown(); // rclcpp 종료
    return 0;
}
