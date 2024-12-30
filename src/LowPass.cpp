#include "LowPass.h"

namespace control {  

LowPass::LowPass(double alpha) : alpha(alpha), prevOutput(0.0) {}

double LowPass::filter(double input) {
    double output = this->alpha * input + (1.0 - this->alpha) * this->prevOutput;
    prevOutput = output;
    return output;
}

}