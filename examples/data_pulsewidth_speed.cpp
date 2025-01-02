#include <iostream>
#include <typeinfo>
#include <string>
#include <thread>
#include "../control.h"

using namespace control;

int main() {

    gpio::setupGpioPinout();

    gpio::reset();

    Motor motor;
    motor.definePin(13);
    motor.definePulseWidthRange(1.0, 1.5, 2.0);

    Velocimeter velocimeter;
    velocimeter.definePin(17);
    velocimeter.defineWheelDiameter(0.105);
    velocimeter.defineAlpha(0.9);

    Writer writer("anchoDePulso_Velocidad.csv", "ancho de pulso (ms), velocidad (m/s)");
    std::vector<std::string> row = {"",""};
        
    std::cout << "Inicio." << std::endl;    
    
    int j = 0;
    for (double i = 1.5; i < 2.0; i += 0.01)
    {  
        motor.setPulseWidth(i);
        std::cout << "pw: " << i <<  std::endl;
        if (i < 1.57) j = 9;
        else j = 0;
        for (j; j < 10; j++)
        {
            velocimeter.start();
            velocimeter.waitForUpdate();
            
            row[0] = std::to_string(i);
            row[1] = std::to_string(velocimeter.getSpeed());
            writer.write_row(row);
        }
    }
    std::cout << "Fin." << std::endl;    
    
    motor.setPulseWidth(motor.pulseWidth.steady);

    std::cout << "velocimeter distancia: " << velocimeter.getDistance() << std::endl;

    return 0;
}
