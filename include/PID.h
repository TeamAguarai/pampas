#ifndef PID_H
#define PID_H

#include <stdexcept>

class PID {
private:
    double kp = 0, ki = 0, kd = 0;
    double minOutput, maxOutput;
    double integral, prevError; 
    bool defined = false;
public:
    bool isDefined();
    void define(double kp, double ki, double kd, double minOutput, double maxOutput);
    double calculate(double setpoint, double measuredValue);
};

#endif
