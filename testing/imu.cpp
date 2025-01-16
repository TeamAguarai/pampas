#include "IMU.h"
#include <iostream>

int main() {
    try {
        IMU sensor;

        while (true) {
            auto [ax, ay, az] = sensor.getAccel();
            auto [gx, gy, gz] = sensor.getGyro();
            auto [mx, my, mz] = sensor.getMag();
            float temp = sensor.getTemp();

            std::cout << "Accel: X=" << ax << " Y=" << ay << " Z=" << az << std::endl;
            std::cout << "Gyro: X=" << gx << " Y=" << gy << " Z=" << gz << std::endl;
            std::cout << "Mag:  X=" << mx << " Y=" << my << " Z=" << mz << std::endl;
            std::cout << "Temp: " << temp << " C" << std::endl;

            delay(500); // Delay for readability
        }

    } catch (const std::exception &e) {
        std::std::cerr << e.what() << std::endl;
        return -1;
    }

    return 0;
}