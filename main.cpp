#include "gpio.h"
#include "Motor.h"
#include "Velocimeter.h"
#include <iostream>

int main() {

    gpio::setupGpioPinout();

    Motor motor;
    motor.definePin(18);
    motor.definePulseWidthRange(1.0, 1.5, 2);

    

    Velocimeter velocimeter;
    velocimeter.definePin(17);
    velocimeter.defineWheelDiameter(0.105);
    velocimeter.start();

    // motor.runForMilliseconds(1000, 1.6);

    double speed; // Variable para almacenar la velocidad ingresada por el usuario

    while (true) {
        std::cout << "Ingrese la velocidad (double): ";
        std::cin >> speed; // Leer la velocidad desde la consola

        // Validar que la velocidad esté dentro del rango permitido
        if (speed >= 1.0 && speed <= 2.0) {
            motor.setSpeed(speed); // Establecer la velocidad
        } else {
            std::cerr << "Error: La velocidad debe estar entre 1.0 y 2.0.\n";
        }
    }

    


    return 0;
}
