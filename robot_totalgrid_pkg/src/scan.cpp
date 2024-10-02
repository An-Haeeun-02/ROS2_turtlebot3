#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath> // std::isnan을 사용하기 위해 추가

class DistanceNode : public rclcpp::Node {
public:
    DistanceNode() : Node("distance_node") {
        auto qos = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&DistanceNode::topic_callback, this, std::placeholders::_1));

        publisher_aeeun_ = this->create_publisher<geometry_msgs::msg::Vector3>("aeeun", 10);
        publisher_hyewon_ = this->create_publisher<geometry_msgs::msg::Vector3>("hyewon", 10);
        publisher_dongwan_ = this->create_publisher<geometry_msgs::msg::Vector3>("dongwan", 10);
    }

private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 원하는 각도 설정 (라디안)
        float angle_0_rad = 0.0;
        float angle_30_rad = M_PI / 6;  // 30도
        float angle_60_rad = M_PI / 3;  // 60도
        float angle_90_rad = M_PI / 2;  // 90도
        float angle_270_rad = 3.0 * M_PI / 2;  // 270도
        float angle_300_rad = 5.0 * M_PI / 3;  // 300도
        float angle_330_rad = 11.0 * M_PI / 6; // 330도

        // 각도에 해당하는 인덱스 계산
        int index_0 = static_cast<int>((angle_0_rad - msg->angle_min) / msg->angle_increment);
        int index_30 = static_cast<int>((angle_30_rad - msg->angle_min) / msg->angle_increment);
        int index_60 = static_cast<int>((angle_60_rad - msg->angle_min) / msg->angle_increment);
        int index_90 = static_cast<int>((angle_90_rad - msg->angle_min) / msg->angle_increment);
        int index_270 = static_cast<int>((angle_270_rad - msg->angle_min) / msg->angle_increment);
        int index_300 = static_cast<int>((angle_300_rad - msg->angle_min) / msg->angle_increment);
        int index_330 = static_cast<int>((angle_330_rad - msg->angle_min) / msg->angle_increment);

        // 거리 계산 (미터를 센티미터로 변환)
        float distance_0_cm = msg->ranges[index_0] * 100.0;
        float distance_30_cm = msg->ranges[index_30] * 100.0;
        float distance_60_cm = msg->ranges[index_60] * 100.0;
        float distance_90_cm = msg->ranges[index_90] * 100.0;
        float distance_270_cm = msg->ranges[index_270] * 100.0;
        float distance_300_cm = msg->ranges[index_300] * 100.0;
        float distance_330_cm = msg->ranges[index_330] * 100.0;

        if (std::isnan(distance_0_cm) || distance_0_cm == 0.00 || std::abs(distance_0_cm) < 1e-10 ||
            std::isnan(distance_90_cm) || distance_90_cm == 0.00 || std::abs(distance_90_cm) < 1e-10 ||
            std::isnan(distance_270_cm) || distance_270_cm == 0.00 || std::abs(distance_270_cm) < 1e-10) {
                RCLCPP_WARN(this->get_logger(), "비정상적인 센서 값이 감지되었습니다.");
        } else {
            // 모든 값이 유효할 때만 퍼블리시
            RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 0 degrees: %.2f cm", distance_0_cm);
            RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 90 degrees: %.2f cm", distance_90_cm);
            RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 270 degrees: %.2f cm", distance_270_cm);

            auto message_aeeun = geometry_msgs::msg::Vector3();
            message_aeeun.x = distance_0_cm;
            message_aeeun.y = distance_90_cm;
            message_aeeun.z = distance_270_cm;

            publisher_aeeun_->publish(message_aeeun);
}


        // hyewon 토픽 메시지 생성 및 퍼블리시
        if (!std::isnan(distance_30_cm) && distance_30_cm != 0.00 &&
            !std::isnan(distance_60_cm) && distance_60_cm != 0.00) {

            RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 30 degrees: %.2f cm", distance_30_cm);
            RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 60 degrees: %.2f cm", distance_60_cm);

            auto message_hyewon = geometry_msgs::msg::Vector3();
            message_hyewon.x = distance_30_cm;
            message_hyewon.y = distance_60_cm;
            message_hyewon.z = 0.0;  // z 값은 사용하지 않으므로 0으로 설정

            publisher_hyewon_->publish(message_hyewon);
        }

        // dongwan 토픽 메시지 생성 및 퍼블리시
        if (!std::isnan(distance_300_cm) && distance_300_cm != 0.00 &&
            !std::isnan(distance_330_cm) && distance_330_cm != 0.00) {

            RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 300 degrees: %.2f cm", distance_300_cm);
            RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 330 degrees: %.2f cm", distance_330_cm);

            auto message_dongwan = geometry_msgs::msg::Vector3();
            message_dongwan.x = distance_300_cm;
            message_dongwan.y = distance_330_cm;
            message_dongwan.z = 0.0;  // z 값은 사용하지 않으므로 0으로 설정

            publisher_dongwan_->publish(message_dongwan);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_aeeun_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_hyewon_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_dongwan_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}
