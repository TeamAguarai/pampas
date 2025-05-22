#ifdef VSCODE_INTELLISENSE_SUPPORT
#include "Governor.hpp"
#include "dev.hpp"
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
    float kp_inicial = askInput("Ingrese valor inicial de Kp: ");
    float ki_inicial = askInput("Ingrese valor inicial de Ki: ");
    float kd_inicial = askInput("Ingrese valor inicial de Kd: ");
    float tau = askInput("Ingrese tau (filtro paso bajo): ");
    float salida_min_pid = askInput("Ingrese salida min PID: ");
    float salida_max_pid = askInput("Ingrese salida max PID: ");
    float integral_min = askInput("Ingrese integral min PID: ");
    float integral_max = askInput("Ingrese integral max PID: ");
    float gamma_p = askInput("Ingrese gamma_p (Adaptativo): ");
    float gamma_i = askInput("Ingrese gamma_i (Adaptativo): ");
    float gamma_d = askInput("Ingrese gamma_d (Adaptativo): ");
    float min_kp = askInput("Ingrese Kp minimo (Adaptativo): ");
    float min_ki = askInput("Ingrese Ki minimo (Adaptativo): ");
    float min_kd = askInput("Ingrese Kd minimo (Adaptativo): ");
    float max_kp = askInput("Ingrese Kp maximo (Adaptativo): ");
    float max_ki = askInput("Ingrese Ki maximo (Adaptativo): ");
    float max_kd = askInput("Ingrese Kd maximo (Adaptativo): ");
    float omega_n = askInput("Ingrese omega_n (MD): ");
    float zeta = askInput("Ingrese zeta (MD): ");
    float beta = askInput("Ingrese beta (2DOF): ");
    float gamma = askInput("Ingrese gamma (2DOF): ");

    AdaptivePID pid;
    pid.setGains(kp_inicial, ki_inicial, kd_inicial); // valores iniciales
    pid.setParameters(tau, salida_min_pid, salida_max_pid, integral_min, integral_max);
    pid.setAdaptiveGains(gamma_p, gamma_i, gamma_d);
    pid.setKpRange(min_kp, max_kp);
    pid.setKiRange(min_ki, max_ki);
    pid.setKdRange(min_kd, max_kd);
    pid.setReferenceModel(omega_n, zeta);
    pid.setTwoDofGains(beta, gamma);

    // --- Velocimeter setup ---
    Velocimeter velocimeter;
    velocimeter.setPin(17); 
    velocimeter.setWheelDiameter(0.105f);  // meters
    velocimeter.setAlpha(0.9f);

    // --- Conversion setup ---
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
