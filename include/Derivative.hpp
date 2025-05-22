/*
Derivative.hpp

Basic class for numerical differentiation of a signal using the backward difference method.
Intended for use in discrete-time control systems or real-time processing.

*/

class Derivative {
public:
    float compute(float value, float dt);
    void reset();
    float get();

private:
    float prev_value_ = 0.0f;
};


/* Computes and returns the derivative using the backward difference method: (current - previous) / dt */
float Derivative::compute(float value, float dt) {
    float derivative = (value - prev_value_) / dt;
    prev_value_ = value;
    return derivative;
}

/* Resets the stored previous value to zero */
void Derivative::reset() {
    prev_value_ = 0.0f;
}

/* Returns the most recently stored input value */
float Derivative::get() {
    return prev_value_;
}
