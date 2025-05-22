/* 
PID.hpp
Creditos: https://github.com/pms67/PID/blob/master/PID.h 

Clase de controlador PID clasico.
Expresion: PID = Kp * (beta * setpoint - measured) + Ki * integral_error + Kd * (gamma * setpoint - measured - prev_output) / dt

Ejemplo de uso:
PID pid_controller;
pid_controller.setGains(...);
pid_controller.setParameters(...);
while (loop) {
    controlled_value = pid_controller.calculate(...);
}

*/

#include <stdexcept>

#ifdef VSCODE_INTELLISENSE_SUPPORT
#include "Exception.hpp"
#endif

namespace pampas {    


class PID {
private:
   
    float derivative_low_pass_filter_constant_;    

    float min_output_;
    float max_output_;

    float min_integral_;
    float max_integral_;

    float prev_error_;
    float integral_;
    
    bool gains_defined_ = false;
    bool params_defined_ = false;

    float kp_;     
    float ki_;     
    float kd_;  

public:
    PID();

    /* Getters */
    float getKp();
    float getKi();
    float getKd();
    float calculate(float setpoint, float measurement, float sample_time);

    /* Setters */
    void setGains(float kp, float ki, float kd);
    void setParameters(float derivative_low_pass_filter_constant, float min_output, float max_output, float min_output_int, float max_output_int);
    void reset();
    
};

PID::PID() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}

void PID::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}

float PID::calculate(float setpoint, float measurement, float sample_time_) {
    if (!gains_defined_) EXCEPTION("Faltan definir las ganancias del controlador PID");
    if (!params_defined_) EXCEPTION( "Faltan definir los parametros del controlador PID");

    float error = setpoint - measurement;
    float proportional = kp_ * error;

    integral_ += (error + prev_error_) *  sample_time_;
    if (integral_ > max_integral_) {
        integral_ = max_integral_;
    } else if (integral_ < min_integral_) {
        integral_ = min_integral_;
    }

    float derivative = (error - prev_error_) / sample_time_;

    float out = proportional + integral_ + derivative;

    if (out > max_output_) {
        out = max_output_;
    } else if (out < min_output_) {
        out = min_output_;
    }

    prev_error_ = error;

    return out;
}

void PID::setGains(float kp, float ki, float kd) {
    gains_defined_ = true;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setParameters(float derivative_low_pass_filter_constant, float min_output_, float max_output_, float min_integral_, float max_integral_) {
    params_defined_ = true;
    derivative_low_pass_filter_constant_ = derivative_low_pass_filter_constant;
    min_output_ = min_output_;
    max_output_ = max_output_;
    min_integral_ = min_integral_;
    max_integral_ = max_integral_;
}

float PID::getKp() {
    return kp_;
}

float PID::getKi() {
    return ki_;
}

float PID::getKd() {
    return kd_;
}

}