#include <rclcpp/rclcpp.hpp> // ROS2 C++ 클라이언트 라이브러리 헤더 파일 포함
#include <sensor_msgs/msg/laser_scan.hpp> // Lidar 센서 데이터 메시지 타입 헤더 파일 포함
#include <iostream> // 입출력 스트림을 사용하기 위한 헤더 파일
#include <vector> // 벡터 컨테이너를 사용하기 위한 헤더 파일
#include <cmath> // 수학 함수를 사용하기 위한 헤더 파일
#include <thread> // 스레드 관련 기능을 위한 헤더 파일
#include <chrono> // 시간 측정을 위한 헤더 파일

using namespace std; // 표준 네임스페이스 사용

class LidarVisualizer : public rclcpp::Node // ROS2 노드 클래스를 정의
{
public:
    LidarVisualizer() : Node("lidar_visualizer") // 생성자: 노드 이름을 'lidar_visualizer'로 설정
    {
        // QoS 설정을 SensorDataQoS로 설정
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); // 최근 10개의 메시지를 유지하고, best_effort 전송 모드 설정
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&LidarVisualizer::topic_callback, this, std::placeholders::_1));
            // '/scan' 토픽을 구독하고, 메시지가 도착하면 topic_callback 함수를 호출
    }

private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) // /scan 토픽의 콜백 함수
    {
        const int grid_width = 200;  // 2D 배열의 가로 크기를 100으로 설정
        const int grid_height = 100;  // 2D 배열의 세로 크기를 50으로 설정
        const int center_x = grid_width / 2;  // 2D 배열의 가로 중심점 계산
        const int center_y = grid_height / 2; // 2D 배열의 세로 중심점 계산

        // 빈 2D 배열 생성 (grid_height x grid_width 크기)
        vector<vector<char>> grid(grid_height, vector<char>(grid_width, ' '));

        // Lidar 데이터 처리
        for (size_t i = 0; i < msg->ranges.size(); ++i) // 각 범위 데이터에 대해 반복
        {
            double angle = msg->angle_min + i * msg->angle_increment; // 현재 각도를 라디안 단위로 계산
            double range = msg->ranges[i] * 100; // 거리 값을 미터에서 센티미터로 변환 (1미터 = 100센티미터)
            
            if (range < msg->range_min * 100 || range > msg->range_max * 100) { // 거리 값이 유효 범위 내에 있는지 확인
                continue; // 유효하지 않으면 건너뜀
            }

            // 좌표 변환 시, Lidar 센서의 방향을 고려한 변환
            double x = range * cos(angle); // 극좌표를 직교 좌표로 변환 (x 좌표)
            double y = range * sin(angle); // 극좌표를 직교 좌표로 변환 (y 좌표)

            // 화면 좌표계로 변환 (중심점을 기준으로)
            int grid_x = static_cast<int>(center_x - y); // x 좌표를 화면 좌표계로 변환
            int grid_y = static_cast<int>(center_y - x); // y 좌표를 화면 좌표계로 변환

            if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height) // 좌표가 배열 범위 내에 있는지 확인
            {
                grid[grid_y][grid_x] = '*'; // 측정된 지점을 배열에 '*'로 표시
            }
        }

        grid[center_y][center_x] = 'O'; // 그리드 중심에 터틀봇을 'O'로 표시

        // 테두리 출력
        for (int i = 0; i < grid_width + 2; ++i) cout << "*"; // 윗변 테두리
        cout << endl;

        // 2D 배열을 터미널에 출력
        for (const auto& row : grid) // 각 행에 대해 반복
        {
            cout << "*"; // 좌측 테두리
            for (const auto& cell : row) // 각 셀에 대해 반복
            {
                cout << cell; // 셀의 값을 출력
            }
            cout << "*" << endl; // 우측 테두리 및 줄 바꿈
        }

        for (int i = 0; i < grid_width + 2; ++i) cout << "*"; // 아랫변 테두리
        cout << endl; // 줄 바꿈

        // 3초 동안 대기
        std::this_thread::sleep_for(std::chrono::seconds(3)); // 3초 동안 스레드를 일시 정지
    }

    // 구독자를 위한 멤버 변수
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // ROS2 시스템 초기화
    rclcpp::spin(std::make_shared<LidarVisualizer>()); // LidarVisualizer 노드를 실행
    rclcpp::shutdown(); // ROS2 시스템 종료
    return 0; // 프로그램 종료
}
