/*
Credits: https://github.com/pms67/PID/blob/master/PID.h

Classic PID controller class.
Expression: PID = Kp * (setpoint - measured) + Ki * ∫error + Kd * d(error)/dt

Example usage:
PID pid_controller;
pid_controller.setGains(...);
pid_controller.setParameters(...);
while (loop) {
    controlled_value = pid_controller.calculate(...);
}
*/

#include <stdexcept>
#include <algorithm>

#ifdef VSCODE_INTELLISENSE_SUPPORT
#include "Exception.hpp"
#include "Derivative.hpp"
#include "Integral.hpp"
#include "LowPass.hpp"
#endif

namespace pampas {

class PID {
protected:
    float derivative_low_pass_filter_constant_;
    float min_output_;
    float max_output_;
    float min_integral_;
    float max_integral_;

    bool gains_defined_ = false;
    bool params_defined_ = false;

    float kp_;
    float ki_;
    float kd_;

    Derivative ErrorDerivative_;
    Integral ErrorIntegral_;
    LowPass<float> DerivativeFilter_;

    float beta_;   // For 2DOF PID
    float gamma_;  // For 2DOF PID

public:
    PID();

    float getKp();
    float getKi();
    float getKd();

    float calculate(float setpoint, float measurement, float sample_time);

    void setGains(float kp, float ki, float kd);
    void setParameters(float derivative_low_pass_filter_constant, float min_output, float max_output, float min_integral, float max_integral);
    void reset();

    enum class Type {
        CLASSIC,
        ADAPTIVE
    };
};

// --- Implementation ---

/* Default constructor: initializes internal states and resets integrator/derivator */
PID::PID():DerivativeFilter_() {
    DerivativeFilter_.setInitialValue(0);
    ErrorDerivative_.reset();
    ErrorIntegral_.reset();
}

/* Resets the internal integral and derivative state */
void PID::reset() {
    ErrorDerivative_.reset();
    ErrorIntegral_.reset();
}

/* Calculates the PID control output for a given setpoint and measurement */
float PID::calculate(float setpoint, float measurement, float sample_time_) {
    if (!gains_defined_) EXCEPTION("PID gains must be defined before calculation.");
    if (!params_defined_) EXCEPTION("PID parameters must be defined before calculation.");

    float error = setpoint - measurement;

    float proportional = kp_ * error;
    float integral = std::clamp(ki_ * ErrorIntegral_.compute(error, sample_time_), min_integral_, max_integral_);
    float derivative = DerivativeFilter_.filter(kd_ * ErrorDerivative_.compute(error, sample_time_));

    std::cout << "\n\nPID Math: " << proportional << " + " << integral << " + " << derivative << "\n";

    float out = std::clamp(proportional + integral + derivative, min_output_, max_output_);

    return out;
}

/* Sets the PID gain values (Kp, Ki, Kd) */
void PID::setGains(float kp, float ki, float kd) {
    gains_defined_ = true;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

/* Sets the controller output and integral limits along with the low-pass filter constant */
void PID::setParameters(float derivative_low_pass_filter_constant, float min_output, float max_output, float min_integral, float max_integral) {
    params_defined_ = true;
    derivative_low_pass_filter_constant_ = derivative_low_pass_filter_constant;
    DerivativeFilter_.setAlpha(derivative_low_pass_filter_constant);
    min_output_ = min_output;
    max_output_ = max_output;
    min_integral_ = min_integral;
    max_integral_ = max_integral;
}

/* Returns the proportional gain (Kp) */
float PID::getKp() {
    return kp_;
}

/* Returns the integral gain (Ki) */
float PID::getKi() {
    return ki_;
}

/* Returns the derivative gain (Kd) */
float PID::getKd() {
    return kd_;
}

} // namespace pampas
