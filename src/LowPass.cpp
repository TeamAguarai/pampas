#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #include "LowPass.h"
#endif

#ifdef PAMPAS_LIBRARY
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