#include <iostream>
#include <string>
#include <thread>
#include "aguarai.h"

int main(int argc, char* argv[]) {

    if (argc <= 1) {
        std::cout << "Error. Debe proporcionar un pin." << std::endl;
        return 0;
    }

    gpio::setupGpioPinout();

    int pin = std::atoi(argv[1]);

    Velocimeter velocimeter;
    velocimeter.definePin(pin);
    velocimeter.defineWheelDiameter(0.105);
    velocimeter.start();

    while (true) {
        std::cout << "\rDistancia Pin(" << pin << "): " << velocimeter.getDistance() << " metros." << std::flush;
    }


    return 0;
}
