#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <iostream>
#include <signal.h>

using namespace std;

class OdomDistanceCalculator : public rclcpp::Node
{
public:
  OdomDistanceCalculator() : Node("odom_distance_calculator"), total_distance_{0.0}, initialized_{false}
  {
    // /odom 토픽을 구독합니다.
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->handle_odom(msg);
      }
    );

    // 로그를 찍기 위한 타이머 설정
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), // 시간 단위 리터럴 수정
      [this]() {
        RCLCPP_INFO(this->get_logger(), "Total Distance: %.2f meters", total_distance_);
      }
    );
  }

private:
  void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 첫 번째 메시지 수신 시 초기화
    if (!initialized_) {
      last_position_ = msg->pose.pose.position;
      initialized_ = true;
      return;
    }

    // 현재 위치 추출
    auto current_position = msg->pose.pose.position;

    // 이전 위치와 현재 위치 간의 거리 계산
    double dx = current_position.x - last_position_.x;
    double dy = current_position.y - last_position_.y;
    double dz = current_position.z - last_position_.z;

    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 총 이동 거리 업데이트
    total_distance_ += distance;
    if (total_distance_ >= 10)

    // 현재 위치를 이전 위치로 업데이트
    last_position_ = current_position;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Point last_position_;
  double total_distance_;
  bool initialized_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<OdomDistanceCalculator>());
  rclcpp::shutdown();
  return 0;
}
