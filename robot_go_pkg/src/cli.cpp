#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_action/action/move.hpp"
#include <iostream>
#include <vector>

class MoveActionClient : public rclcpp::Node
{
public:
    using Move = robot_action::action::Move;
    using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

    MoveActionClient() : Node("Move_action_client"), goal_handle_(nullptr)
    {
        // "an"이라는 이름의 액션 클라이언트 생성
        this->client_ = rclcpp_action::create_client<Move>(
            this,
            "an"
        );

        // 액션 서버가 사용 가능할 때까지 5초 기다림
       this->client_->wait_for_action_server();

        // 사용자 입력을 처리하는 함수 호출
        this->process_user_input();
    }

private:
    rclcpp_action::Client<Move>::SharedPtr client_;
    rclcpp_action::ClientGoalHandle<Move>::SharedPtr goal_handle_;

    void process_user_input()
    {
        while (rclcpp::ok()) {
            // 사용자로부터 이동 거리 입력 받기
            float distance;
            std::cout << "Enter the distance to move: ";
            std::cin >> distance;

            // 목표 메시지 생성 및 설정
            auto goal_msg = Move::Goal();
            goal_msg.distance = distance; // 필드 이름 수정

            // 목표 옵션 생성
            auto goal_options = rclcpp_action::Client<Move>::SendGoalOptions();

            // 목표 응답 콜백 정의
            goal_options.goal_response_callback = [this](std::shared_ptr<GoalHandleMove> goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Goal accepted");
                this->goal_handle_ = goal_handle;
            };

            // 피드백 콜백 정의
            goal_options.feedback_callback = [this](GoalHandleMove::SharedPtr, const std::shared_ptr<const Move::Feedback> feedback) {
                std::string distances_str;
                for (float distance : feedback->traveled_distances) {
                    distances_str += std::to_string(distance) + " ";
                }
                RCLCPP_INFO(this->get_logger(), "Received feedback: traveled distances = [%s]", distances_str.c_str());
            };

            // 결과 콜백 정의
            goal_options.result_callback = [this](const GoalHandleMove::WrappedResult &result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Result received: success = %s", result.result->success ? "true" : "false");
                } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
                    RCLCPP_ERROR(this->get_logger(), "Result was aborted");
                } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    RCLCPP_ERROR(this->get_logger(), "Result was cancelled");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Result code: UNKNOWN");
                }
            };

            // 비동기 목표 전송
            auto goal_handle_future = client_->async_send_goal(goal_msg, goal_options);

            // 목표 전송 완료를 기다리고 처리
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
                return;
            }

            auto goal_handle = goal_handle_future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "No goal handle");
                return;
            }

            // 결과를 기다리거나 추가 작업을 수행할 수 있음
            // 사용자가 새로운 거리를 입력할 수 있도록 준비
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveActionClient>());
    rclcpp::shutdown();
    return 0;
}
