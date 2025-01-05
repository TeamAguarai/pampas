#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #include "PulseWidth.h"
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
#endif
namespace pampas {
    
void PulseWidth::set(double min, double steady, double max) 
{ 
    this->min = min; 
    this->max = max;
    this->steady = steady; 
}

bool PulseWidth::isDefined() 
{
    if (this->min == -1 || this->max == -1 || this->steady == -1) return false;
    return true;
}

double PulseWidth::validate(double pulseWidth) 
{
    if (pulseWidth > this->max) return this->max;
    if (pulseWidth < this->min) return this->min;
    return pulseWidth;
}

}