/*
LowPass.hpp

Class for applying a low-pass filter.

Example usage:
LowPass low_pass_filter;
low_pass_filter.setInitialValue(...);
low_pass_filter.setAlpha(...);
low_pass_filter.filter(value);
*/

#ifdef VSCODE_INTELLISENSE_SUPPORT
#include "Exception.hpp"
#endif

namespace pampas {

template<typename T>
class LowPass {
private:
    T alpha_;              // Smoothing coefficient (must be set before use)
    T prev_output_;        // Previous filtered output
    bool alpha_defined_ = false;

public:
    LowPass();
    void setInitialValue(T value);
    void setAlpha(T value);
    T filter(T input);
};

// --- Implementation ---

/* Constructor: initializes the previous output to zero */
template<typename T>
LowPass<T>::LowPass() : prev_output_(static_cast<T>(0)) {}

/* Sets the smoothing coefficient; must be in range [0, 1] */
template<typename T>
void LowPass<T>::setAlpha(T value) {
    if (value < static_cast<T>(0) || value > static_cast<T>(1)) {
        throw EXCEPTION("Smoothing coefficient must be in the range [0, 1]");
    }
    alpha_ = value;
    alpha_defined_ = true;
}

/* Sets the initial output value of the filter */
template<typename T>
void LowPass<T>::setInitialValue(T value) {
    prev_output_ = value;
}

/* Applies the low-pass filter to the input */
template<typename T>
T LowPass<T>::filter(T input) {
    if (!alpha_defined_) {
        throw EXCEPTION("Smoothing coefficient has not been set or is invalid.");
    }

    T output = alpha_ * input + (static_cast<T>(1) - alpha_) * prev_output_;
    prev_output_ = output;
    return output;
}

// Explicit instantiations
template class LowPass<double>;
template class LowPass<float>;
template class LowPass<int>;

} // namespace pampas
