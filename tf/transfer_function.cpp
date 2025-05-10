#include <iostream>
#include <typeinfo>
#include <string>
#include <thread>
#include "pampas.h"

using namespace pampas;

int main() {

    gpio::setupGpioPinout();

    Motor motor;
    motor.setPin(18);
    motor.setPulseWidthRange(1.0, 1.5, 2.0);

    Velocimeter velocimeter;
    velocimeter.setPin(17);
    velocimeter.setWheelDiameter(0.105);
    velocimeter.setAlpha(0.9);

    Writer writer("anchoDePulso_Velocidad_xd.csv", "ancho de pulso (ms), velocidad (m/s)");
    std::vector<std::string> row = {"",""};
        
    std::cout << "Inicio." << std::endl;    
    
    float adp;
    std::cout << "Ingrese el ancho de pulso (milli-segundos): ";
    std::cin >> adp;

    int segundos;
    std::cout << "Ingrese cantidad de tiempo a medir (segundos): ";
    std::cin >> segundos;

    int lecturas = 0;

    auto inicio = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - inicio < std::chrono::seconds(segundos)) {
        lecturas++;
        motor.setPulseWidth(adp);
        velocimeter.start();
        velocimeter.waitForUpdate();
        
        row[0] = std::to_string(adp);
        row[1] = std::to_string(velocimeter.getSpeed());
        writer.write_row(row);
    }
    std::cout << "Fin." << std::endl;    
    std::cout << "Lecturas realizadas: " << lecturas << std::endl;    

    
    motor.setPulseWidth(motor.pulseWidth.steady);


    return 0;
}
