#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath>

class DistanceNode : public rclcpp::Node {
public:
    DistanceNode() : Node("distance_node") {
        auto qos = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&DistanceNode::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("aeeun", 10);
    }

private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 원하는 각도 설정 (라디안)
        float angle_0_rad = 0.0;
        float angle_90_rad = M_PI / 2;
        float angle_270_rad = 3.0 * M_PI / 2;

        // 각도에 해당하는 인덱스 계산
        int index_0 = static_cast<int>((angle_0_rad - msg->angle_min) / msg->angle_increment);
        int index_90 = static_cast<int>((angle_90_rad - msg->angle_min) / msg->angle_increment);
        int index_270 = static_cast<int>((angle_270_rad - msg->angle_min) / msg->angle_increment);

        // 해당 인덱스의 거리 계산 (미터를 센티미터로 변환)
        float distance_0_cm = msg->ranges[index_0] * 100.0;
        float distance_90_cm = msg->ranges[index_90] * 100.0;
        float distance_270_cm = msg->ranges[index_270] * 100.0;

        RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 0 degrees: %.2f cm", distance_0_cm);
        RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 90 degrees: %.2f cm", distance_90_cm);
        RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 270 degrees: %.2f cm", distance_270_cm);

        auto message = geometry_msgs::msg::Vector3();
        message.x = distance_0_cm;
        message.y = distance_90_cm;
        message.z = distance_270_cm;

        publisher_->publish(message);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}