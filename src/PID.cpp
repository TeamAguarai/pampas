#ifdef USING_VSCODE_AS_EDITOR
    #include "PID.h"
#endif

/* Creditos: https://github.com/pms67/PID/blob/master/PID.c */
namespace pampas {

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
    this->gains_defined_ = true;
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;
}

void PID::setParameters(double tau_, double min_output_, double max_output_, double min_output_int_, double max_output_int_) 
{
    this->params_defined_ = true;
    this->tau_ = tau_;
    this->min_output_ = min_output_;
    this->max_output_ = max_output_;
    this->min_output_int_ = min_output_int_;
    this->max_output_int_ = max_output_int_;
}

}