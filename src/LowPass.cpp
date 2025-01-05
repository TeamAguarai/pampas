#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #include "LowPass.h"
#endif

#ifdef CONTROL_LIBRARY
    #include "pampas.h"
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