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

        // 라이다 센서의 감지 범위 정의
        const double MIN_DISTANCE_CM = 12.00;   // 최소 감지 거리 (12cm)
        const double MAX_DISTANCE_CM = 350.00; // 최대 감지 거리 (350cm/3.5m)

        // 조건을 그룹화하여 간결하게 표현
        if (std::isnan(distance_0_cm) || std::isnan(distance_90_cm) || std::isnan(distance_270_cm)) {
            RCLCPP_WARN(this->get_logger(), "(%.2fcm): NaN 값이 감지되었습니다.",distance_0_cm);
        } else if ((distance_0_cm == 0.00 || std::abs(distance_0_cm) < 1e-10) ||
                (distance_90_cm == 0.00 || std::abs(distance_90_cm) < 1e-10) ||
                (distance_270_cm == 0.00 || std::abs(distance_270_cm) < 1e-10)) {
            RCLCPP_WARN(this->get_logger(), "(%.2fcm): 0 또는 너무 작은 값이 감지되었습니다.",distance_0_cm);
        } else if (distance_0_cm < 12.00 || distance_90_cm < 12.00 || distance_270_cm < 12.00) {
            RCLCPP_WARN(this->get_logger(), "(%.2fcm): 최소 거리보다 작은 값이 감지되었습니다.", distance_0_cm);
        } else if (distance_0_cm > 350.00 || distance_90_cm > 500.00 || distance_270_cm > 500.00) {
            RCLCPP_WARN(this->get_logger(), "(%.2fcm): 최대 거리보다 큰 값이 감지되었습니다.", distance_0_cm);
        } else {
            // 0도 방향 값만 유효하면 출력 및 퍼블리시
            RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 0 degrees: %.2f cm", distance_0_cm);

            auto message_aeeun = geometry_msgs::msg::Vector3();
            message_aeeun.x = distance_0_cm;
            message_aeeun.y = 0.0;  // 나머지 값은 사용하지 않으므로 0으로 설정
            message_aeeun.z = 0.0;
            publisher_aeeun_->publish(message_aeeun);
        }


        // hyewon 토픽 메시지 생성 및 퍼블리시
        if (!std::isnan(distance_30_cm) && distance_30_cm != 0.00 &&
            !std::isnan(distance_60_cm) && distance_60_cm != 0.00) {

            //RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 30 degrees: %.2f cm", distance_30_cm);
            //RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 60 degrees: %.2f cm", distance_60_cm);

            auto message_hyewon = geometry_msgs::msg::Vector3();
            message_hyewon.x = distance_30_cm;
            message_hyewon.y = distance_60_cm;
            message_hyewon.z = 0.0;  // z 값은 사용하지 않으므로 0으로 설정

            publisher_hyewon_->publish(message_hyewon);
        }

        // dongwan 토픽 메시지 생성 및 퍼블리시
        if (!std::isnan(distance_300_cm) && distance_300_cm != 0.00 &&
            !std::isnan(distance_330_cm) && distance_330_cm != 0.00) {

            //RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 300 degrees: %.2f cm", distance_300_cm);
            //RCLCPP_INFO(this->get_logger(), "Distance to obstacle at 330 degrees: %.2f cm", distance_330_cm);

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

    //일반값인 소숫점 두자리수가 넘으면 잘못된 값으로 인식
    bool is_valid_value(double value) {
    // 소수점 이하 자릿수를 확인하기 위해 문자열로 변환
    std::ostringstream oss;
    oss << value;
    std::string value_str = oss.str();

    // 소수점 위치 찾기
    size_t dot_pos = value_str.find('.');
    if (dot_pos == std::string::npos) {
        return false; // 소수점이 없는 경우 잘못된 숫자로 간주
    }

    // 소수점 뒤에 실제 자릿수를 계산
    size_t decimal_places = value_str.length() - dot_pos - 1;

    // 소수점 뒤의 자릿수가 정확히 2인지 확인
    if (decimal_places == 2)
    {
        return false;
    }else
    {
        return true;
    }
}
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}