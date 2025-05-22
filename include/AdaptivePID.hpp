/*
AdaptivePID.hpp

Adaptive PID controller class.
Extends a classic PID controller by adding gain adaptation and optional two-degree-of-freedom (2DOF) control.

Requires prior configuration of base PID gains and parameters before use.
*/

#ifdef VSCODE_INTELLISENSE_SUPPORT
#include "PID.hpp"
#include "Derivative.hpp"
#include "Exception.hpp"
#endif

#include <algorithm>
#include <stdexcept>
#include <string>

namespace pampas {

class AdaptivePID : public PID {
private:
    /* Adaptative gains */
    float Gp_;
    float Gi_;
    float Gd_;
    float max_kp_;
    float max_ki_;
    float max_kd_;
    float min_kp_;
    float min_ki_;
    float min_kd_;

    /* 2 DOF PID */
    float beta_;
    float gamma_;
    
    /* Rerefence Model */
    float omega_n_;
    float zeta_;
    double dv_m_;
    double reference_model_setpoint_;
    
    /* Flags */
    bool adaptive_gains_defined_;
    bool two_dof_gains_defined_;
    bool reference_model_gains_defined_;
    bool adaptive_kp_range_defined_;
    bool adaptive_ki_range_defined_;
    bool adaptive_kd_range_defined_;

    Derivative ReferenceModelDerivate_;
    Derivative PlantDerivative_;

public:
    AdaptivePID();

    float getBeta();
    float getGamma();
    float calculate(float setpoint, float measurement, float sample_time);
    
    float setReferenceModel(float omega_n, float zeta);
    void setAdaptiveGains(float Gp, float Gi, float Gd);
    void setKpRange(float min_value, float max_value);
    void setKiRange(float min_value, float max_value);
    void setKdRange(float min_value, float max_value);
    void setTwoDofGains(float beta, float gamma);
};

AdaptivePID::AdaptivePID():ReferenceModelDerivate_(), PlantDerivative_() {
    bool adaptive_gains_defined_ = false;
    bool two_dof_gains_defined_ = false;
    bool reference_model_gains_defined_ = false;
    bool adaptive_kp_range_defined_ = false;
    bool adaptive_ki_range_defined_ = false;
    bool adaptive_kd_range_defined_ = false;

    kp_ = 0;
    ki_ = 0;
    kd_ = 0;

    dv_m_ = 0;
    reference_model_setpoint_ = 0;
}

/* Computes the control output using adaptive PID logic with optional 2DOF support */
float AdaptivePID::calculate(float setpoint, float measurement, float sample_time_) {
    if (!gains_defined_) EXCEPTION("PID gains must be defined before calculation.");
    if (!params_defined_) EXCEPTION("PID parameters must be defined before calculation.");
    if (!adaptive_kp_range_defined_ || !adaptive_ki_range_defined_ || !adaptive_kd_range_defined_) EXCEPTION("Adaptive gains must be defined before calculation.");
    
    /* Reference model output */
    double ddv_m = (omega_n_ * omega_n_) * setpoint - 2.0 * zeta_ * omega_n_ * dv_m_ - (omega_n_ * omega_n_) * reference_model_setpoint_;
    dv_m_ += ddv_m * sample_time_;
    reference_model_setpoint_ += dv_m_ * sample_time_;
    
    ReferenceModelDerivate_.compute(reference_model_setpoint_, sample_time_);
    PlantDerivative_.compute(measurement, sample_time_);
    float D_err = gamma_ * ReferenceModelDerivate_.get() - PlantDerivative_.get();

    float error = reference_model_setpoint_ - measurement;

    /* Adaptive PID */
    kp_ += Gp_ * error * (beta_*setpoint - measurement) * sample_time_;
    ki_ += Gi_ * error * error * sample_time_;
    kd_ += Gd_ * error * D_err * sample_time_;
    std::clamp(kp_, min_kp_, max_kp_);
    std::clamp(ki_, min_ki_, max_ki_);
    std::clamp(kd_, min_kd_, max_kd_);

    float proportional = kp_ * (beta_*setpoint - measurement);
    float integral_ = ki_ * ErrorIntegral_.compute(error, sample_time_, min_integral_, max_integral_);
    float derivative = kd_ * ErrorDerivative_.compute(error, sample_time_);

    float out = proportional + integral_ + derivative;
    std::clamp(out, min_output_, max_output_);

    return out;
}

/* Sets the adaptation gains for Kp, Ki, and Kd */
void AdaptivePID::setAdaptiveGains(float Gp, float Gi, float Gd) {
    Gp_ = Gp;
    Gi_ = Gi;
    Gd_ = Gd;
    adaptive_gains_defined_ = true;
}

/* Sets the beta and gamma coefficients for 2DOF PID behavior */
void AdaptivePID::setTwoDofGains(float beta, float gamma) {
    beta_ = beta;
    gamma_ = gamma;
    two_dof_gains_defined_ = true;
}

float AdaptivePID::setReferenceModel(float omega_n, float zeta) {
    omega_n_ = omega_n;
    zeta_ = zeta;
    reference_model_gains_defined_ = true;
}

void AdaptivePID::setKpRange(float min_value, float max_value) {
    adaptive_kp_range_defined_ = true;
    min_kp_ = min_value;
    max_kp_ = max_value;
}

void AdaptivePID::setKpRange(float min_value, float max_value) {
    adaptive_ki_range_defined_ = true;
    min_ki_ = min_value;
    max_ki_ = max_value;
}

void AdaptivePID::setKpRange(float min_value, float max_value) {
    adaptive_kd_range_defined_ = true;
    min_kd_ = min_value;
    max_kd_ = max_value;
}

/* Returns the beta coefficient used in 2DOF PID control */
float AdaptivePID::getBeta() {
    return beta_;
}

/* Returns the gamma coefficient used in 2DOF PID control */
float AdaptivePID::getGamma() {
    return gamma_;
}


} // namespace pampas
