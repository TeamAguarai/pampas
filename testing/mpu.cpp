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
    const std::string calibrationFile = "calibration.txt";
    pampas::MPU9250 IMU;
    std::cout << "NO CALIBRACION MPU" << std::endl;
    // Inicializar el sensor
    int status = IMU.begin();
    if (status < 0) {
        cerr << "Error: Inicialización del IMU fallida." << endl;
        return -1;
    }

    // Manejar calibración (se puede sobrescribir si es necesario)
    // if (!IMU.handleCalibration(true)) { // Cambiar a `true` para sobrescribir
    if (!IMU.handleCalibration(false)) { // Cambiar a `true` para sobrescribir
        cerr << "Error: No se pudo completar la calibración." << endl;
        return -1;
    }

    // Leer datos y calcular yaw en un bucle infinito
    while (true) {
        IMU.readSensor();

        // Obtener datos del sensor
        float ax = IMU.getAccelX_mss();
        float ay = IMU.getAccelY_mss();
        float az = IMU.getAccelZ_mss();
        float mx = IMU.getMagX_uT();
        float my = IMU.getMagY_uT();
        float mz = IMU.getMagZ_uT();

        // Calcular yaw
        float yaw = calculateYaw(ax, ay, az, mx, my, mz);

        // Mostrar los valores
        cout << fixed << setprecision(2);
        cout << "Acelerómetro (m/s²): X=" << ax << " Y=" << ay << " Z=" << az << endl;
        cout << "Magnetómetro (uT):   X=" << mx << " Y=" << my << " Z=" << mz << endl;
        cout << "Yaw (°): " << yaw << endl;

        pampas::delay(500); // Esperar 500 ms entre lecturas
    }

    return 0;
}
