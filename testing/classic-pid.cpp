#ifdef VSCODE_INTELLISENSE_SUPPORT
#include "Governor.hpp"
#include "PID.hpp"
#endif

#include "pampas.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace pampas;

int main() {
    // --- Motor setup ---
    Motor motor;
    motor.setPulseWidthRange(1.0f, 1.5f, 2.0f); // min, steady, max
    motor.setPin(13); 

    // --- PID setup ---
    float kp = askInput("Ingrese kp: ");
    float ki = askInput("Ingrese ki: ");
    float kd = askInput("Ingrese kd: ");
    float tau = askInput("Ingrese tau (filtro paso bajo): ");
    float salida_min_pid = askInput("Ingrese salida min PID: ");
    float salida_max_pid = askInput("Ingrese salida max PID: ");
    float integral_min = askInput("Ingrese integral min: ");
    float integral_max = askInput("Ingrese integral max: ");
    PID pid;
    pid.setGains(kp, ki, kd);
    pid.setParameters(tau, salida_min_pid, salida_max_pid, integral_min, integral_max);

    // --- Velocimeter setup ---
    Velocimeter velocimeter;
    velocimeter.setPin(17); // dummy pin
    velocimeter.setWheelDiameter(0.105f);  // meters
    velocimeter.setAlpha(0.9f);

    // --- Conversion setup (simple lineal: x => x) ---
    Conversion ms_to_pwm;
    ms_to_pwm.set([](double value) -> double {
        return 1.5406368187143187 
        + (0.2630704132809448) * pow(x, 1)
        + (-0.7839709920881821) * pow(x, 2)
        + (1.1329846648132278) * pow(x, 3)
        + (-0.9313254302627445) * pow(x, 4)
        + (0.4795173149457668) * pow(x, 5)
        + (-0.1621289088299355) * pow(x, 6)
        + (0.0365891255894518) * pow(x, 7)
        + (-0.0054579880129316) * pow(x, 8)
        + (0.0005162340433127) * pow(x, 9)
        + (-0.0000280155501921) * pow(x, 10)
        + (0.0000006638052154) * pow(x, 11);
    });

    // --- Governor ---
    Governor governor(motor, pid, velocimeter, ms_to_pwm);

    governor.run(2.5f);  // velocidad deseada en m/s

    return 0;
}
