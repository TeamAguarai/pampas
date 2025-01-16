#include <iostream>
#include <string>
#include <thread>
#include "../aguarai.h"

int main() {

    gpio::setupGpioPinout();

    Velocimeter velocimeter;
    velocimeter.setPin(17);
    velocimeter.setWheelDiameter(0.105);
    velocimeter.start();

    std::cout << "Inicio" << std::endl;
    while (true) {
        std::cout << "\rVelocidad Pin(" << 17 << "): " << velocimeter.getSpeed() << " m/s" << std::flush;
        gpio::delay(10);
    }


    return 0;
}
