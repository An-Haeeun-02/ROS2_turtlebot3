#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/empty.hpp"
#include <csignal>
#include <atomic>
#include <cmath>
#include <iostream>

using namespace std;

// 시그널 핸들러 (Ctrl+C로 종료 시 호출됨)
void signalHandler(int /*signum*/)
{
    cout << "Ctrl+C로 종료합니다." << endl;

    // 로봇을 정지시키기 위한 메시지 생성
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.angular.z = 0.0;

    // 로봇을 정지시키기 위해 cmd_vel 토픽에 메시지 발행
    auto node = rclcpp::Node::make_shared("stop_publisher");
    auto pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // 메시지 발행
    pub->publish(message);

    // 메시지가 발행될 시간을 줌
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    rclcpp::shutdown();
}

class DirectNode : public rclcpp::Node
{
public:
  DirectNode() : Node("direct_node"), initial_orientation_(0.0)
  {
    // 퍼블리셔와 서브스크라이버 설정
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&DirectNode::imuCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&DirectNode::odomCallback, this, std::placeholders::_1));

    // 서비스 서버 설정
    stop_service_ = this->create_service<std_srvs::srv::Empty>(
      "/stop_robot", std::bind(&DirectNode::stopServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 기본 직진 속도 설정
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DirectNode::controlLoop, this));
  }

  void spin()
  {
    while (rclcpp::ok())
    {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // IMU에서 현재 로봇의 방향을 가져옴
    tf2::Quaternion quat;
    tf2::fromMsg(msg->orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    current_orientation_ = yaw;

    if (!is_initialized_)
    {
      initial_orientation_ = yaw;  // 로봇 시작 방향을 기억
      is_initialized_ = true;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr /*msg*/)
  {
    // 오도메트리 데이터를 활용하여 이동 상태를 확인 가능 (필요에 따라)
  }

  void stopServiceCallback(const std_srvs::srv::Empty::Request::SharedPtr /*request*/,
                           const std_srvs::srv::Empty::Response::SharedPtr /*response*/)
  {
    stop_requested_ = true;  // 로봇 멈추기 플래그 설정
  }

  void controlLoop()
  {
    if (!is_initialized_ || stop_requested_)
    {
      // 로봇 멈추기
      auto cmd_msg = geometry_msgs::msg::Twist();
      cmd_msg.linear.x = 0.0;
      cmd_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_msg);
      return;
    }

    // 현재 각도와 초기 각도의 차이 계산
    double angle_diff = current_orientation_ - initial_orientation_;

    // 각도 차이를 [-π, π] 범위로 정규화
    while (angle_diff > M_PI)
      angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI)
      angle_diff += 2 * M_PI;

    // Twist 메시지 생성
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = 0.2;  // 기본 직진 속도

    // 각도 차이에 따라 회전 속도 조정
    if (std::fabs(angle_diff) > 0.05)  // 각도 차이가 0.05 라디안 이상일 때만 보정
    {
      cmd_msg.angular.z = -angle_diff * 1.0;  // 각도 차이에 비례하여 회전 속도 설정
      cmd_msg.linear.x = 0.1;  // 회전 중일 때는 직진 속도 줄임
    }
    else
    {
      cmd_msg.angular.z = 0.0;  // 각도 차이가 작으면 회전하지 않음
    }

    // 명령 퍼블리시
    cmd_vel_pub_->publish(cmd_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  double initial_orientation_;
  double current_orientation_;
  bool is_initialized_ = false;
  bool stop_requested_ = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectNode>();

  // 시그널 핸들러 등록
  signal(SIGINT, signalHandler);

  node->spin();  // 노드 실행
  rclcpp::shutdown();
  return 0;
}