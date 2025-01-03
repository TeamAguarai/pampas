#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #pragma once
#endif

#ifdef CONTROL_LIBRARY
    #include "control.h"
#endif

/* Creditos: https://github.com/pms67/PID/blob/master/PID.h */

#include <stdexcept>

namespace control {    


class PID {
private:
    double Kp;     
    double Ki;     
    double Kd;     
    double tau;    

    double minOutput;
    double maxOutput;

    double minOutputInt;
    double maxOutputInt;

    double sampleTime;

    double integrator = 0;
    double prevError = 0;
    double differentiator = 0;
    double prevMeasurement = 0;

    double out;

    bool gainsDefined = false;
    bool paramsDefined = false;

public:

    PID();
    void setGains(double kp, double ki, double kd);
    void setParameters(double tau, double minOutput, double maxOutput, double minOutputInt, double maxOutputInt);
    double calculate(double setpoint, double measurement, double sampleTime);
};

}