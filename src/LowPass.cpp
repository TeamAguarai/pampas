#include "LowPass.h"

namespace control {  


void LowPass::defineAlpha(double value) 
{
    this->alpha = value;
}


double LowPass::filter(double input) {
    double output = this->alpha * input + (1.0 - this->alpha) * this->prevOutput;
    prevOutput = output;
    return output;
}

}