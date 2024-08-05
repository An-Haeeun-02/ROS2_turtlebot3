#include <iostream>
#include <libgpsmm.h>
#include <cmath>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class GPSNode : public rclcpp::Node {
public:
    GPSNode() : Node("gps_node") {
        // Publisher를 초기화합니다.
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("gps", 10);

        // Timer를 설정하여 주기적으로 콜백 함수를 호출합니다.
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GPSNode::publish_gps_data, this)
        );

        // GPSD 서버와의 연결을 설정합니다.
        gps_rec_ = new gpsmm("localhost", DEFAULT_GPSD_PORT);
        if (gps_rec_->stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
            RCLCPP_ERROR(this->get_logger(), "GPSD 서버가 실행되고 있지 않습니다.");
        }
    }

    ~GPSNode() {
        delete gps_rec_;
    }

private:
    void publish_gps_data() {
        struct gps_data_t* gps_data;

        // GPS 데이터를 읽어옵니다.
        if ((gps_data = gps_rec_->read()) == NULL) {
            RCLCPP_ERROR(this->get_logger(), "데이터 읽기 오류.");
            return;
        }

        // GPS 데이터가 유효한지 확인하고 메시지를 발행합니다.
        if (gps_data->fix.mode >= MODE_2D &&
            !std::isnan(gps_data->fix.latitude) &&
            !std::isnan(gps_data->fix.longitude)) {

            auto msg = geometry_msgs::msg::Vector3();
            msg.x = gps_data->fix.latitude;
            msg.y = gps_data->fix.longitude;
            msg.z = 0.0;  // 높이 값은 사용할 수 없으므로 0으로 설정합니다.

            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "위도: %f, 경도: %f", msg.x, msg.y);
        } else {
            RCLCPP_WARN(this->get_logger(), "GPS 신호 없음.");
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    gpsmm* gps_rec_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPSNode>());
    rclcpp::shutdown();
    return 0;
}
