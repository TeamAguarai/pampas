/* Creditos: https://github.com/pms67/PID/blob/master/PID.cpp */

#include "PID.h"


PID::PID() 
{
    integrator = 0.0f;
    prevError = 0.0f;
    differentiator = 0.0f;
    prevMeasurement = 0.0f;
    out = 0.0f;
}

double PID::calculate(double setpoint, double measurement, double sampleTime) 
{
    if (!gainsDefined) throw std::invalid_argument( "Faltan definir las ganancias del controlador PID" );
    if (!paramsDefined) throw std::invalid_argument( "Faltan definir los parametros del controlador PID" );

    double error = setpoint - measurement;
    double proportional = Kp * error;
    integrator += 0.5f * Ki * sampleTime * (error + prevError);

    if (integrator > limMaxInt) {
        integrator = limMaxInt;
    } else if (integrator < limMinInt) {
        integrator = limMinInt;
    }

    // Derivativo (filtro de paso bajo)
    differentiator = -(2.0f * Kd * (measurement - prevMeasurement) +
                       (2.0f * tau - sampleTime) * differentiator) /
                     (2.0f * tau + sampleTime);

    out = proportional + integrator + differentiator;

    if (out > limMax) {
        out = limMax;
    } else if (out < limMin) {
        out = limMin;
    }

    prevError = error;
    prevMeasurement = measurement;

    return out;
}

void PID::setGains(double kp, double ki, double kd) 
{
    this->gainsDefined = true;
    this->Kp = kp;
    this->Ki = ki;
    this->Kd = kd;
}

void PID::setParameters(double tau, double limMin, double limMax, double limMinInt, double limMaxInt) 
{
    this->paramsDefined = true;
    this->tau = tau;
    this->limMin = limMin;
    this->limMax = limMax;
    this->limMinInt = limMinInt;
    this->limMaxInt = limMaxInt;
}