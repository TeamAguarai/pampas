#include <iostream>
#include <string>
#include <thread>
#include "pampas.h"

using namespace pampas;

int main() {

    gpio::setupGpioPinout();

    Velocimeter velocimeter;
    velocimeter.setPin(17);
    velocimeter.setWheelDiameter(0.105/2);
    velocimeter.setAlpha(1.0);
    velocimeter.start();

    std::cout << "Inicio" << std::endl;
    while (true) {
        std::cout << "\rVelocidad Pin(" << 17 << "): " << velocimeter.getSpeed() << " m/s" << std::flush;
        pampas::delay(10);
    }


    return 0;
}
