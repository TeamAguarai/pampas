/*
Integral.hpp

Basic class for numerical integration using the rectangular method.
Includes clamping to prevent integral wind-up.

*/

class Integral {
public:
    float compute(float currentValue, float dt, float min, float max);
    void reset();
    float get() const;

private:
    float accumulated_ = 0.0f;
};

// --- Implementation ---

/* Computes and returns the integral using the rectangular method.
   Accumulates the product of current value and dt.
   Clamps the result between min and max to prevent wind-up. */
float Integral::compute(float currentValue, float dt, float min, float max) {
    accumulated_ += currentValue * dt;
    if (accumulated_ > max) accumulated_ = max;
    if (accumulated_ < min) accumulated_ = min;
    return accumulated_;
}

/* Resets the accumulated integral to zero. */
void Integral::reset() {
    accumulated_ = 0.0f;
}

/* Returns the current accumulated value. */
float Integral::get() const {
    return accumulated_;
}
