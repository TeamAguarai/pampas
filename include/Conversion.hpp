/*
Conversion.hpp

Class for converting between pulse width (PWM in nanoseconds) and velocity (m/s) using a transfer function.

Example usage:
Conversion conversion;
conversion.set(least_square_function);
data = conversion.convert(value);

ToDo: improve error message when function is not defined.
*/

#ifdef VSCODE_INTELLISENSE_SUPPORT
#include "Exception.hpp"
#endif 

#include <functional>
#include <stdexcept>
#include <string>

namespace pampas {

class Conversion {
public:
    void set(std::function<double(double)> func);
    double convert(double input);
    bool isDefined();

private:
    std::function<double(double)> transfer_function_;
    bool defined_ = false;
    
};

/* Returns whether a conversion function has been defined */
bool Conversion::isDefined() {
    return defined_;
}

/* Sets the conversion transfer function */
void Conversion::set(std::function<double(double)> func) {
    defined_ = true;
    transfer_function_ = func;
}

/* Applies the conversion function to the input value */
double Conversion::convert(double input) {
    if (!defined_) EXCEPTION("Conversion function is not defined.");
    return transfer_function_(input);
}

} // namespace pampas
