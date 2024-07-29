#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <iostream>
#include "robot_action/action/move.hpp"

using namespace std;

class MoveClient : public rclcpp::Node {
public:
    using Move = robot_action::action::Move;
    using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

    MoveClient()
        : Node("move_client")
    {
        client_ = rclcpp_action::create_client<Move>(this, "an");
        wait_for_action_server();
    }

    void process_user_input()
    {
        while (rclcpp::ok()) {
            float distance;
            std::cout << "이동할 거리를 입력하세요: ";
            std::cin >> distance;

            auto goal_msg = Move::Goal();
            goal_msg.distance = distance;

            auto goal_options = rclcpp_action::Client<Move>::SendGoalOptions();
            
            goal_options.goal_response_callback = [this](std::shared_ptr<GoalHandleMove> goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "목표가 거부되었습니다");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "목표가 수락되었습니다");
            };

            goal_options.feedback_callback = [](GoalHandleMove::SharedPtr, const std::shared_ptr<const Move::Feedback> feedback) {
                std::string distances_str;
                for (float distance : feedback->traveled_distances) {
                    distances_str += std::to_string(distance) + " ";
                }
                RCLCPP_INFO(rclcpp::get_logger("move_client"), "피드백 수신: 이동한 거리 = [%s]", distances_str.c_str());
            };

            goal_options.result_callback = [](const GoalHandleMove::WrappedResult &result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(rclcpp::get_logger("move_client"), "결과 수신: 성공 여부 = %s", result.result->success ? "참" : "거짓");
                } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
                    RCLCPP_ERROR(rclcpp::get_logger("move_client"), "결과가 중단되었습니다");
                } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    RCLCPP_ERROR(rclcpp::get_logger("move_client"), "결과가 취소되었습니다");
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("move_client"), "결과 코드: 알 수 없음");
                }
            };

            auto goal_handle_future = client_->async_send_goal(goal_msg, goal_options);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(), "목표 전송 실패");
                return;
            }

            // 목표를 보낸 후에 콜백을 실시간으로 처리하기 위해 spin_some을 주기적으로 호출
            auto goal_handle = goal_handle_future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "서버가 목표를 거부했습니다");
                return;
            }

            auto result_future = client_->async_get_result(goal_handle);

            while (rclcpp::ok()) {
                rclcpp::spin_some(this->get_node_base_interface());

                if (result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                    break;
                }
            }

            auto result = result_future.get();
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "목표 성공");
            } else {
                RCLCPP_ERROR(this->get_logger(), "목표 실패: 코드 %d", static_cast<int>(result.code));
            }
        }
    }

private:
    rclcpp_action::Client<Move>::SharedPtr client_; // 클라이언트 변수 선언

    void wait_for_action_server()
    {
        while (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(this->get_logger(), "액션 서버를 기다리는 중...");
        }
        RCLCPP_INFO(this->get_logger(), "액션 서버가 사용 가능합니다.");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveClient>();
    node->process_user_input();
    rclcpp::shutdown();

    return 0;
}