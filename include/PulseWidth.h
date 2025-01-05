#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #pragma once
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
#endif

namespace pampas {
    
class PulseWidth
{
public:
    double min = -1; // -1 como valor indefinido
    double max = -1;
    double steady = -1;
    void set(double min, double steady, double max);
    bool isDefined();
    double validate(double pulseWidthMs);
};

}