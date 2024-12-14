#include "PID.h"

PID::PID(double kp, double ki, double kd, double minOutput, double maxOutput)
    : kp(kp), ki(ki), kd(kd), minOutput(minOutput), maxOutput(maxOutput), integral(0.0), prevError(0.0) {}

double PID::calculate(double setpoint, double measuredValue) {
    double error = setpoint - measuredValue;

    double proportional = kp * error;

    integral += error; 
    double integralComponent = ki * integral;

    double derivative = error - prevError; 
    double derivativeComponent = kd * derivative;

    prevError = error;

    double output = proportional + integralComponent + derivativeComponent;

    if (output > maxOutput) output = maxOutput;
    else if (output < maxOutput) output = minOutput;
    
    return output;
}
