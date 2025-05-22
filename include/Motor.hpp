/*
Motor.hpp

Class to control the motor of the HYPER VS 1/8 BUGGY NITRO.

Example usage:
Motor motor;
motor.setPin(...);
motor.setPulseWidthRange(...);
motor.setPulseWidth(value);
*/

#ifdef VSCODE_INTELLISENSE_SUPPORT
#include "PulseWidth.hpp"
#include "gpio.hpp"
#include "Exception.hpp"
#endif

#include <csignal>
#include <stdexcept>

namespace pampas {

class Motor {
public:
    Motor();
    void setPin(int pin);
    void setPulseWidthRange(float min, float steady, float max);
    void setPulseWidth(float pulseWidth);
    void steady();

    PulseWidth pulseWidth_;

private:
    int pin_ = -1;
};

// --- Implementation ---

/* Default constructor */
Motor::Motor() {}

/* Sets the motor pin to the steady PWM value */
void Motor::steady() {
    setPulseWidth(pulseWidth_.steady_);
}

/* Sets the GPIO pin used for PWM output */
void Motor::setPin(int pin) {
    pin_ = pin;
    gpio::pinMode(pin_, PWM_OUTPUT);
}

/* Sets the minimum, steady, and maximum pulse widths */
void Motor::setPulseWidthRange(float min, float steady, float max) {
    pulseWidth_.set(min, steady, max);
}

/* Applies a validated PWM pulse width to the motor */
void Motor::setPulseWidth(float pulseWidth) {
    if (!pulseWidth_.isDefined()) throw EXCEPTION("Pulse width values must be defined.");
    if (pin_ == -1) throw EXCEPTION("Motor pin has not been set.");
    gpio::pwmWrite(pin_, pulseWidth_.validate(pulseWidth));
}

} // namespace pampas
