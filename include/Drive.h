#ifndef DRIVE_H
#define DRIVE_H

#include "Motor.h"
#include "Velocimeter.h"
#include "PID.h"
#include "Conversion.h"
#include "gpio.h"
#include <thread>
#include <functional>

class Drive {
private:
    bool running = false;
    std::thread control;
    Motor motor;
    Velocimeter velocimeter;
    PID pid; // el metodo pid.define debe de llamarse antes de mover al motor
    Conversion MsToPulseWidth; // el metodo pulseWidthToMs.define debe de llamarse antes de mover al motor
    double runningSpeed = 0;
public:
    Drive();
    void definePid(double kp, double ki, double kd, double minOutput, double maxOutput);
    void defineTransferFunction(std::function<double(double)> func);
    void run(double speed);
    void stop();
};

#endif