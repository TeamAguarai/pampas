#include <iostream>
#include <iomanip>
#include <cmath> // Para atan2

#ifdef USING_VSCODE_AS_EDITOR
    #include "MPU9250.h"
#else
    #include "pampas.h"
#endif

using namespace std;

// Función para calcular yaw
float calculateYaw(float ax, float ay, float az, float mx, float my, float mz) {
    // Calcular roll y pitch desde el acelerómetro
    float roll = atan2(ay, az); // Roll (inclinación sobre X)
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)); // Pitch (inclinación sobre Y)

    // Compensar el magnetómetro
    float mx_comp = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch);
    float my_comp = my * cos(roll) - mz * sin(roll);

    // Calcular yaw usando atan2
    float yaw = atan2(my_comp, mx_comp) * 180.0 / M_PI; // Convertir a grados

    // Ajustar el rango del ángulo a [0, 360]
    if (yaw < 0) {
        yaw += 360.0;
    }

    return yaw;
}

int main() {
    pampas::MPU9250 IMU;
    // Inicializar el sensor
    int status = IMU.begin();
    if (status < 0) {
        cerr << "Error: Inicialización del IMU fallida. Codigo: " << status << endl;

        return -1;
    }

    IMU.calibrate();

    // std::cout << "Mag Bias X (µT): " << IMU.getMagBiasX_uT() << std::endl;
    // std::cout << "Mag Scale Factor X: " << IMU.getMagScaleFactorX() << std::endl;
    // std::cout << "Mag Bias Y (µT): " << IMU.getMagBiasY_uT() << std::endl;
    // std::cout << "Mag Scale Factor Y: " << IMU.getMagScaleFactorY() << std::endl;
    // std::cout << "Mag Bias Z (µT): " << IMU.getMagBiasZ_uT() << std::endl;
    // std::cout << "Mag Scale Factor Z: " << IMU.getMagScaleFactorZ() << std::endl;

    // std::cout << "Calibracion acelerometro. Mover en figura de 8" << std::endl;
    // IMU.calibrateAccel();
    // std::cout << "Calibracion end." << std::endl;
    
    // std::cout << "Accel Bias X (m/s²): " << IMU.getAccelBiasX_mss() << std::endl;
    // std::cout << "Accel Scale Factor X: " << IMU.getAccelScaleFactorX() << std::endl;
    // std::cout << "Accel Bias Y (m/s²): " << IMU.getAccelBiasY_mss() << std::endl;
    // std::cout << "Accel Scale Factor Y: " << IMU.getAccelScaleFactorY() << std::endl;
    // std::cout << "Accel Bias Z (m/s²): " << IMU.getAccelBiasZ_mss() << std::endl;
    // std::cout << "Accel Scale Factor Z: " << IMU.getAccelScaleFactorZ() << std::endl;
    // Manejar calibración (se puede sobrescribir si es necesario)
    // if (!IMU.handleCalibration(true)) { // Cambiar a `true` para sobrescribir
    // if (!IMU.handleCalibration(false)) { // Cambiar a `true` para sobrescribir
    //     cerr << "Error: No se pudo completar la calibración." << endl;
    //     return -1;
    // }

    // Leer datos y calcular yaw en un bucle infinito
    int c = 0;
    while (true) {
        IMU.readSensor();
        IMU.updateAngles();
        c++;
        std::cout << "\n\nLectura [" << c << "]: " << std::endl;
    //     std::cout << "📌 Datos del Acelerómetro (m/s²):" << std::endl;
    //     std::cout << "  ➤ X: " << IMU.getAccelX_mss() << " m/s²" << std::endl;
    //     std::cout << "  ➤ Y: " << IMU.getAccelY_mss() << " m/s²" << std::endl;
    //     std::cout << "  ➤ Z: " << IMU.getAccelZ_mss() << " m/s²" << std::endl;

        std::cout << "\n📌 Datos del Giroscopio (rad/s):" << std::endl;
        std::cout << "  ➤ X: " << IMU.getGyroX_rads()  << " rad/s" << std::endl;
        std::cout << "  ➤ Y: " << IMU.getGyroY_rads()  << " rad/s" << std::endl;
        std::cout << "  ➤ Z: " << IMU.getGyroZ_rads()  << " rad/s" << std::endl;
        std::cout << "  ➤ Angle X axis: " << IMU.getThetaX() << "°" << std::endl;
        std::cout << "  ➤ Angle Y axis: " << IMU.getThetaY() << "°" << std::endl;



        pampas::delay(300); // Esperar 500 ms entre lecturas
    }

    return 0;
}
