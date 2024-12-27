#include "PID.h"

bool PID::isDefined() 
{
    return this->defined;
}

void PID::define(double kp, double ki, double kd, double minOutput, double maxOutput) 
{
    if (kp < 0 || ki < 0 || kd < 0) {
        throw std::invalid_argument("Las constantes del PID deben ser mayores o iguales a cero.");
    }

    this->kp = kp;
    this->ki = ki; 
    this->kd = kd, 
    this->minOutput = minOutput, 
    this->maxOutput = maxOutput, 
    this->integral = 0;
    this->prevError = 0;

    this->defined = true;
}

double PID::calculate(double setpoint, double measuredValue) 
{
    if (!this->defined) throw std::invalid_argument("Constantes PID no definidas.");

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
