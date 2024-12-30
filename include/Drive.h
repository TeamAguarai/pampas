#ifndef DRIVE_H
#define DRIVE_H

#include "Motor.h"
#include "Velocimeter.h"
#include "PID.h"
#include "Conversion.h"
#include "gpio.h"
#include <thread>
#include <omp.h>
#include <functional>

class Drive {
private:
    std::thread control;
    PID pid; // el metodo pid.define debe de llamarse antes de mover al motor
    bool running = false;
    double runningSpeed = 0;
    void controlledSpeed(double speed);
public:
    Conversion MsToPulseWidth; // el metodo pulseWidthToMs.define debe de llamarse antes de mover al motor
    Motor motor;
    Velocimeter velocimeter;
    Drive();
    void run(double speed);
    void stop();

    /* Llamar estos metodos antes de mover al motor */
    void definePid(double kp, double ki, double kd, double tau, double minOutput, double maxOutput, double minOutputInt, double maxOutputInt);
    void defineTransferFunction(std::function<double(double)> func);
    void defineMotor(int pin, double pulseWidthMin, double pulseWidthSteady, double pulseWidthMax);
    void defineVelocimeter(int pin, double wheelDiameter);
};

#endif