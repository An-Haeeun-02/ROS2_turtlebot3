#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <iostream>
#include "robot_action/action/move.hpp"

using namespace std;

class MoveClient : public rclcpp::Node {
public:
    // 액션 타입과 목표 핸들 타입 정의
    using Move = robot_action::action::Move;
    using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

    // 생성자: MoveClient 객체를 초기화
    MoveClient()
        : Node("move_client") // 노드 이름을 "move_client"로 설정
    {
        // 액션 클라이언트를 생성하고, 액션 서버의 이름은 "an"으로 설정
        client_ = rclcpp_action::create_client<Move>(this, "an");
        // 액션 서버가 사용 가능할 때까지 대기
        wait_for_action_server();
    }

    // 사용자 입력을 처리하는 함수
    void process_user_input()
    {
        while (rclcpp::ok()) { // ROS 노드가 종료되지 않는 한 반복
            float distance;
            // 사용자에게 이동할 거리를 입력 받기 위한 메시지 출력
            std::cout << "이동할 거리를 입력하세요: ";
            std::cin >> distance; // 사용자로부터 거리 입력 받기

            // 목표 메시지 생성 및 설정
            auto goal_msg = Move::Goal();
            goal_msg.distance = distance; // 입력 받은 거리를 목표 메시지에 설정

            // 목표 전송 옵션 생성
            auto goal_options = rclcpp_action::Client<Move>::SendGoalOptions();
            
            // 목표 응답 콜백 정의: 목표가 액션 서버에 의해 수락되거나 거부될 때 호출됨
            goal_options.goal_response_callback = [this](std::shared_ptr<GoalHandleMove> goal_handle) {
                if (!goal_handle) {
                    // 목표가 액션 서버에 의해 거부된 경우 에러 메시지 출력
                    std::cerr << "목표가 거부되었습니다" << std::endl;
                    return;
                }
                // 목표가 액션 서버에 의해 수락된 경우 정보 메시지 출력
                std::cout << "목표가 수락되었습니다" << std::endl;
            };

            // 피드백 콜백 정의: 목표 처리 중에 수신되는 피드백에 대한 콜백
            goal_options.feedback_callback = [](GoalHandleMove::SharedPtr, const std::shared_ptr<const Move::Feedback> feedback) {
                std::string distances_str;
                // 피드백에서 이동한 거리들을 문자열로 변환
                for (float distance : feedback->traveled_distances) {
                    distances_str += std::to_string(distance) + " ";
                }
                // 피드백 정보를 정보 메시지로 출력
                std::cout << "피드백 수신: 이동한 거리 = [" << distances_str << "]" << std::endl;
            };

            // 결과 콜백 정의: 목표 처리 결과를 수신했을 때 호출됨
            goal_options.result_callback = [](const GoalHandleMove::WrappedResult &result) {
                // 결과 코드에 따라 적절한 메시지 출력
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    // 목표가 성공적으로 완료된 경우 정보 메시지 출력
                    std::cout << "결과 수신: 성공 여부 = " << (result.result->success ? "참" : "거짓") << std::endl;
                } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
                    // 목표가 중단된 경우 에러 메시지 출력
                    std::cerr << "결과가 중단되었습니다" << std::endl;
                } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    // 목표가 취소된 경우 에러 메시지 출력
                    std::cerr << "결과가 취소되었습니다" << std::endl;
                } else {
                    // 결과 코드가 알 수 없는 경우 에러 메시지 출력
                    std::cerr << "결과 코드: 알 수 없음" << std::endl;
                }
            };

            // 비동기적으로 목표를 전송하고, 목표 핸들을 포함한 future 객체를 반환
            auto goal_handle_future = client_->async_send_goal(goal_msg, goal_options);

            // 목표 전송이 완료될 때까지 기다림
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                // 목표 전송 실패 시 에러 메시지 출력
                std::cerr << "목표 전송 실패" << std::endl;
                return;
            }

            // 목표를 전송한 후 목표 핸들을 가져옴
            auto goal_handle = goal_handle_future.get();
            if (!goal_handle) {
                // 목표가 액션 서버에 의해 거부된 경우 에러 메시지 출력
                std::cerr << "서버가 목표를 거부했습니다" << std::endl;
                return;
            }

            // 비동기적으로 결과를 요청하고, 결과를 포함한 future 객체를 반환
            auto result_future = client_->async_get_result(goal_handle);

            // 결과가 준비될 때까지 대기
            while (rclcpp::ok()) {
                // 콜백 처리를 위해 spin_some 호출
                rclcpp::spin_some(this->get_node_base_interface());

                // 결과가 준비되었는지 확인
                if (result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                    break; // 결과가 준비되면 반복문 종료
                }
            }

            // 결과를 가져옴
            auto result = result_future.get();
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                // 목표가 성공적으로 완료된 경우 정보 메시지 출력
                std::cout << "목표 성공" << std::endl;
            } else {
                // 목표가 실패한 경우 에러 메시지 출력
                std::cerr << "목표 실패: 코드 " << static_cast<int>(result.code) << std::endl;
            }
        }
    }

private:
    rclcpp_action::Client<Move>::SharedPtr client_; // 액션 클라이언트 변수

    // 액션 서버가 사용 가능할 때까지 대기하는 함수
    void wait_for_action_server()
    {
        while (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            // 액션 서버가 사용 가능해질 때까지 5초 간격으로 대기 메시지 출력
            std::cout << "액션 서버를 기다리는 중..." << std::endl;
        }
        // 액션 서버가 사용 가능할 때 정보 메시지 출력
        std::cout << "액션 서버가 사용 가능합니다." << std::endl;
    }
};

// main 함수: ROS 2 노드를 초기화하고 실행
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); // ROS 2 초기화
    auto node = std::make_shared<MoveClient>(); // MoveClient 객체 생성
    node->process_user_input(); // 사용자 입력 처리
    rclcpp::shutdown(); // ROS 2 종료

    return 0;
}