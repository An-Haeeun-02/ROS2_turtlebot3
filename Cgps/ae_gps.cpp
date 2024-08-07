#include <iostream>
#include <libgpsmm.h>
#include <cmath>  // 이 헤더를 추가하세요

int main() {
    gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);

    if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
        std::cerr << "No GPSD running.\n";
        return 1;
    }

    while (true) {
        struct gps_data_t* gps_data;

        if ((gps_data = gps_rec.read()) == NULL) {
            std::cerr << "Read error.\n";
        } else {
            if (gps_data->fix.mode >= MODE_2D &&
                !std::isnan(gps_data->fix.latitude) &&  // std::isnan을 사용합니다
                !std::isnan(gps_data->fix.longitude)) {
                std::cout << "Latitude: " << gps_data->fix.latitude
                          << " Longitude: " << gps_data->fix.longitude << "\n";
            } else {
                std::cout << "No fix.\n";
            }
        }
        sleep(1);
    }
    return 0;
}
