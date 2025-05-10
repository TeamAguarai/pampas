#ifdef USING_VSCODE_AS_EDITOR
    #include "operations.h"
#else
    #include "pampas.h"
#endif

#include <iostream>

int main() {
    double imu_value = 1.23456789; // Ejemplo de lectura del sensor IMU
    double val = 0.004999999888241291046142578125;
    int decimals = 3; // Especificar cuántos decimales queremos

    double rounded_value = pampas::round(imu_value, decimals);
    std::cout << "Valor IMU redondeado: " << rounded_value << std::endl;

    double rounded_value2 = pampas::round(val, 3);
    std::cout << "Valor redondeado: " << rounded_value2 << std::endl;

    return 0;
}