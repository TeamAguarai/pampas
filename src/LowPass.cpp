#ifdef USING_VSCODE_AS_EDITOR
    #include "LowPass.h"
#endif

namespace pampas {  


void LowPass::setAlpha(double value) 
{
    this->alpha = value;
}


double LowPass::filter(double input) {
    double output = this->alpha * input + (1.0 - this->alpha) * this->prevOutput;
    prevOutput = output;
    return output;
}

}