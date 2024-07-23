#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_action/action/move.hpp"

class MoveActionServer : public rclcpp::Node
{
public:
    using Move = robot_action::action::Move;
    using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

    MoveActionServer() : Node("move_action_server")
    {
        this->action_server_ = rclcpp_action::create_server<Move>(
            this,
            "move",
            std::bind(&MoveActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveActionServer::handle_accepted, this, std::placeholders::_1)
        );
    }

private:
    rclcpp_action::Server<Move>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Move::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request: %s", goal->message.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
    {
        std::thread{
            [this, goal_handle]() {
                const auto goal = goal_handle->get_goal();
                auto feedback = std::make_shared<Move::Feedback>();
                auto result = std::make_shared<Move::Result>();

                feedback->feedback = "잘 받았습니다.";
                goal_handle->publish_feedback(feedback);

                // Simulate some processing time
                std::this_thread::sleep_for(std::chrono::seconds(2));

                result->response = "안녕하세요";
                goal_handle->succeed(result);
            }
        }.detach();
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveActionServer>());
    rclcpp::shutdown();
    return 0;
}
