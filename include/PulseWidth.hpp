/*
PulseWidth.hpp

Class for managing the valid range of PWM pulse width values.
Includes validation and scaling from normalized percentage input.

Example usage:
PulseWidth pulsewidth;
pulsewidth.set(...);
in_range = pulsewidth.validate(pulseMs);
scaled = pulsewidth.fromPercentage(0.5f);
*/

#ifdef VSCODE_INTELLISENSE_SUPPORT
#include "Exception.hpp"
#endif

namespace pampas {

class PulseWidth {
private:
    float min_;
    float max_;
    float steady_;
    bool defined_;
public:
    PulseWidth();
    void set(float min, float steady, float max);
    bool isDefined();
    float validate(float pulseWidthMs);
    float fromPercentage(float percent);
};

PulseWidth::PulseWidth() {
    bool defined_ = false;
}

/* Sets the minimum, steady, and maximum allowed pulse widths */
void PulseWidth::set(float min, float steady, float max) {
    min_ = min;
    max_ = max;
    steady_ = steady;
    defined_ = true;
}

/* Returns true if all pulse width values have been defined */
bool PulseWidth::isDefined() {
    return defined_;
}

/* Ensures the given pulse width is within the valid [min, max] range */
float PulseWidth::validate(float pulseWidth) {
    if (!defined_) EXCEPTION("Rango PWM no esta definido para el motor");
    if (pulseWidth > max_) return max_;
    if (pulseWidth < min_) return min_;
    return pulseWidth;
}

/* Converts a normalized percentage [0.0–1.0] to a pulse width value within [min_, max_] */
float PulseWidth::fromPercentage(float percent) {
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 1.0f) percent = 1.0f;
    return min_ + percent * (max_ - min_);
}

} // namespace pampas
