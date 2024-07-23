#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_action/action/move.hpp"

class MoveActionClient : public rclcpp::Node
{
public:
    using Move = robot_action::action::Move;
    using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

    MoveActionClient() : Node("move_action_client")
    {
        this->client_ = rclcpp_action::create_client<Move>(
            this,
            "move"
        );

        this->send_goal();
    }

private:
    rclcpp_action::Client<Move>::SharedPtr client_;

    void send_goal()
    {
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = Move::Goal();
        goal_msg.message = "안녕";

        auto goal_options = rclcpp_action::Client<Move>::SendGoalOptions();
        
        // Goal response callback 정의
        goal_options.goal_response_callback = [](std::shared_ptr<GoalHandleMove> goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was rejected");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted");
        };

        // 피드백 콜백 정의
        goal_options.feedback_callback = [this](GoalHandleMove::SharedPtr, const std::shared_ptr<const Move::Feedback> feedback) {
            RCLCPP_INFO(this->get_logger(), "Received feedback: %s", feedback->feedback.c_str());
        };

        // 결과 콜백 정의
        goal_options.result_callback = [this](const GoalHandleMove::WrappedResult &result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Result received: %s", result.result->response.c_str());
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Result was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Result was cancelled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Result code: UNKNOWN");
                    break;
            }
        };

        auto goal_handle_future = client_->async_send_goal(goal_msg, goal_options);

        // 결과를 기다리고 처리하는 로직 추가
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

        // 결과를 기다리거나 추가 작업을 수행할 수 있습니다.
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveActionClient>());
    rclcpp::shutdown();
    return 0;
}
