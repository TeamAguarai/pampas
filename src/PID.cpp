#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #include "PID.h"
#endif

#ifdef CONTROL_LIBRARY
    #include "control.h"
#endif

/* Creditos: https://github.com/pms67/PID/blob/master/PID.cpp */
namespace control {

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

    if (integrator > maxOutputInt) {
        integrator = maxOutputInt;
    } else if (integrator < minOutputInt) {
        integrator = minOutputInt;
    }

    // Derivativo (filtro de paso bajo)
    differentiator = -(2.0f * Kd * (measurement - prevMeasurement) +
                       (2.0f * tau - sampleTime) * differentiator) /
                     (2.0f * tau + sampleTime);

    out = proportional + integrator + differentiator;

    if (out > maxOutput) {
        out = maxOutput;
    } else if (out < minOutput) {
        out = minOutput;
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

void PID::setParameters(double tau, double minOutput, double maxOutput, double minOutputInt, double maxOutputInt) 
{
    this->paramsDefined = true;
    this->tau = tau;
    this->minOutput = minOutput;
    this->maxOutput = maxOutput;
    this->minOutputInt = minOutputInt;
    this->maxOutputInt = maxOutputInt;
}

}