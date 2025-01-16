#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include "../pampas.h"

std::atomic<bool> running(true); // Controla si el hilo de velocidad sigue activo

// Función para imprimir la velocidad continuamente en un hilo separado
void printSpeedContinuously(Velocimeter& velocimeter) {
    
    while (running) { // Ejecutar mientras el programa esté activo
        double speed = velocimeter.getSpeed(); // Obtener velocidad actual
        std::cout << "\rVelocidad actual: " << speed << " m/s   " << std::flush; // Imprimir en la misma línea
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Esperar 200 ms entre lecturas
    }
}

int main() {
    // Configuración inicial del GPIO
    gpio::setupGpioPinout();

    // Configurar el motor
    Motor motor;
    motor.setPin(13);
    motor.pulseWidth.set(1.5, 1.5, 2.0);

    // Configurar el velocímetro
    Velocimeter velocimeter;
    velocimeter.setPin(17);
    velocimeter.setWheelDiameter(0.105); // Diámetro de la rueda en metros
    velocimeter.start(); // Iniciar medición de velocidad

    // Iniciar el hilo para mostrar la velocidad continuamente
    std::thread speedThread(printSpeedContinuously, std::ref(velocimeter));

    // Escritor para registrar datos (opcional, según sea necesario)
    Writer w("hola", "hola");

    double pulseWidth;

    // Bucle para ingresar pulseWidth
    while (true) {
        std::cout << "\nIngrese el ancho de pulso (pulseWidth) en ms (Ctrl+C para salir): ";
        std::cin >> pulseWidth;

        // Validar el rango del pulseWidth
        if (pulseWidth < 1.0 || pulseWidth > 2.0) {
            std::cout << "\nAncho de pulso inválido. Debe estar entre 1.0 ms y 2.0 ms." << std::endl;
            continue;
        }

        // Establecer el pulseWidth en el motor
        motor.setPulseWidth(pulseWidth);
        std::cout << "\nPulseWidth establecido a " << pulseWidth << " ms" << std::endl;
    }

    // Finalizar el hilo de velocidad (nunca se alcanzará en este ejemplo)
    running = false;
    speedThread.join(); // Esperar a que el hilo termine

    return 0;
}
