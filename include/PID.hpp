/* 
PID.hpp
Creditos: https://github.com/pms67/PID/blob/master/PID.h 

Clase de controlador PID clasico.

Ejemplo de uso:
PID pid_controller;
pid_controller.setGains(...);
pid_controller.setParameters(...);
while (loop) {
    controlled_value = pid_controller.calculate(...);
}

*/

#include <stdexcept>

namespace pampas {    


class PID {
private:
   
    double tau_;    

    double min_output_;
    double max_output_;

    double min_output_int_;
    double max_output_int_;

    double sample_time_;

    double integrator_ = 0;
    double prev_error_ = 0;
    double differentiator_ = 0;
    double prev_measurement_ = 0;

    double out_;

    bool gains_defined_ = false;
    bool params_defined_ = false;

public:
    double kp_;     
    double ki_;     
    double kd_;  

    PID();
    void setGains(double kp, double ki, double kd);
    void reset();
    void setParameters(double tau, double min_output, double max_output, double min_output_int, double max_output_int);
    double calculate(double setpoint, double measurement, double sample_time);
};

PID::PID() 
{
    integrator_ = 0.0f;
    prev_error_ = 0.0f;
    differentiator_ = 0.0f;
    prev_measurement_ = 0.0f;
    out_ = 0.0f;
}

void PID::reset() {
    integrator_ = 0.0f;
    prev_error_ = 0.0f;
    differentiator_ = 0.0f;
    prev_measurement_ = 0.0f;
    out_ = 0.0f;
}

double PID::calculate(double setpoint, double measurement, double sample_time_) 
{
    if (!gains_defined_) throw std::invalid_argument( "Faltan definir las ganancias del controlador PID" );
    if (!params_defined_) throw std::invalid_argument( "Faltan definir los parametros del controlador PID" );

    double error = setpoint - measurement;
    double proportional = kp_ * error;
    integrator_ += 0.5f * ki_ * sample_time_ * (error + prev_error_);

    if (integrator_ > max_output_int_) {
        integrator_ = max_output_int_;
    } else if (integrator_ < min_output_int_) {
        integrator_ = min_output_int_;
    }

    // Derivativo (filtro de paso bajo)
    differentiator_ = -(2.0f * kd_ * (measurement - prev_measurement_) +
                       (2.0f * tau_ - sample_time_) * differentiator_) /
                     (2.0f * tau_ + sample_time_);

    out_ = proportional + integrator_ + differentiator_;

    if (out_ > max_output_) {
        out_ = max_output_;
    } else if (out_ < min_output_) {
        out_ = min_output_;
    }

    prev_error_ = error;
    prev_measurement_ = measurement;

    return out_;
}

void PID::setGains(double kp, double ki, double kd) 
{
    gains_defined_ = true;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setParameters(double tau_, double min_output_, double max_output_, double min_output_int_, double max_output_int_) 
{
    params_defined_ = true;
    tau_ = tau_;
    min_output_ = min_output_;
    max_output_ = max_output_;
    min_output_int_ = min_output_int_;
    max_output_int_ = max_output_int_;
}

}