#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #pragma once
#endif

#ifdef CONTROL_LIBRARY
    #include "control.h"
#endif

namespace control {
    
class PulseWidth
{
public:
    double min = -1; // -1 como valor indefinido
    double max = -1;
    double steady = -1;
    void define(double min, double steady, double max);
    bool isDefined();
    double validate(double pulseWidthMs);
};

}