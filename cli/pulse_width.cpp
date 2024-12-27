#include <iostream>
#include "../aguarai.h"

int main() {
    gpio::setupGpioPinout();

    Motor motor;
    motor.definePin(13);
    motor.pulseWidth.define(1.5, 1.5, 2.0);

    double pulseWidth;
    while (true) {
        std::cout << "Ingrese el ancho de pulso (pulseWidth) en ms (Ctrl+C para salir): ";
        std::cin >> pulseWidth;

        if (pulseWidth < 1.0 || pulseWidth > 2.0) {
            std::cout << "Ancho de pulso inválido. Debe estar entre 1.0 ms y 2.0 ms." << std::endl;
            continue;
        }

        motor.setPulseWidth(pulseWidth);
        std::cout << "PulseWidth establecido a " << pulseWidth << " ms" << std::endl;
    }

    return 0;
}
