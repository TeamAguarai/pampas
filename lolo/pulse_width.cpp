#include <iostream>
#include "pampas.h"

int main() {
    pampas::gpio::setupGpioPinout();

    pampas::Motor motor;
    motor.setPin(18);
    std::cout << "Motor inicializado en PIN " << 18 << "\n";
    motor.pulseWidth.set(1.5, 1.5, 2.0);

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
