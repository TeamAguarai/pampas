#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #include "PulseWidth.h"
#endif

#ifdef CONTROL_LIBRARY
    #include "control.h"
#endif
namespace control {
    
void PulseWidth::define(double min, double steady, double max) 
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