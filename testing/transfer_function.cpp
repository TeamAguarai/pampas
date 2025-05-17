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
    std::vector<std::string> row = {"","",""};
        
    std::cout << "Inicio." << std::endl;    
    
    float adp;
    std::cout << "Ingrese el ancho de pulso (milli-segundos): ";
    std::cin >> adp;

    int segundos;
    std::cout << "Ingrese cantidad de tiempo a medir (segundos): ";
    std::cin >> segundos;

    int lecturas = 0;
    motor.setPulseWidth(adp);

    //double distancia = 2 * (0.105) * 3.1416;
    double distancia = (3.1416 * 0.105) / 2; // para 2 imanes
    auto inicio = std::chrono::steady_clock::now();
    double dt_total = 0;
    while (std::chrono::steady_clock::now() - inicio < std::chrono::seconds(segundos)) {
        lecturas++;
        velocimeter.start();
        velocimeter.waitForUpdate();
        double dt = velocimeter.getUpdateTimeInterval();
        
        row[0] = std::to_string(adp);
        row[1] = std::to_string(velocimeter.getSpeed());
        dt_total += dt;
        row[2] = std::to_string(dt_total);
        writer.write_row(row);
        std::cout << velocimeter.getUpdateTimeInterval() << " * " << distancia << " = " << velocimeter.getUpdateTimeInterval() * distancia << " = " << velocimeter.getSpeed() << "\n" ;
    }
    std::cout << "Fin." << std::endl;    
    std::cout << "Lecturas realizadas: " << lecturas << std::endl;    

    
    motor.setPulseWidth(motor.pulseWidth.steady);


    return 0;
}
