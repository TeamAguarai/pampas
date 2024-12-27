/* Creditos: https://github.com/pms67/PID/blob/master/PID.h */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdexcept>

class PID {
private:
    double Kp;     
    double Ki;     
    double Kd;     
    double tau;    

    double limMin;
    double limMax;

    double limMinInt;
    double limMaxInt;

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
    void setParameters(double tau, double limMin, double limMax, double limMinInt, double limMaxInt);
    double calculate(double setpoint, double measurement, double sampleTime);
};

#endif 
