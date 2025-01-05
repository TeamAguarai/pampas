#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #pragma once

    #include "Motor.h"
    #include "Velocimeter.h"
    #include "PID.h"
    #include "Conversion.h"
    #include "gpio.h"
#endif

#ifdef CONTROL_LIBRARY
    #include "control.h"
#endif


#include <thread>
#include <functional>

namespace control {


class Drive {
private:
    std::thread control;
    control::PID pid; // el metodo pid.set debe de llamarse antes de mover al motor
    bool running = false;
    double runningSpeed = 0;
    void controlledSpeed(double speed);
public:
    control::Conversion MsToPulseWidth; // el metodo pulseWidthToMs.set debe de llamarse antes de mover al motor
    control::Motor motor;
    control::Velocimeter velocimeter;
    Drive();
    void run(double speed);
    void stop();

    /* Llamar estos metodos antes de mover al motor */
    void definePid(double kp, double ki, double kd, double tau, double minOutput, double maxOutput, double minOutputInt, double maxOutputInt);
    void defineTransferFunction(std::function<double(double)> func);
    void defineMotor(int pin, double pulseWidthMin, double pulseWidthSteady, double pulseWidthMax);
    void defineVelocimeter(int pin, double wheelDiameter, double alpha);
};

}