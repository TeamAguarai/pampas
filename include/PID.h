#ifndef PID_H
#define PID_H

class PID {
public:
    PID(double kp, double ki, double kd, double minOutput, double maxOutput);

    double calculate(double setpoint, double measuredValue);

private:
    double kp, ki, kd;
    double minOutput, maxOutput;
    double integral, prevError; 
};

#endif
