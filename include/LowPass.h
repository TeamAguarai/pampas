#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #pragma once
#endif

#ifdef CONTROL_LIBRARY
    #include "pampas.h"
#endif


namespace pampas {
    
class LowPass {
private:
    double alpha;       // Coeficiente de suavizado
    double prevOutput = 0.0;  // Última salida (x_{n-1})

public:
    void setAlpha(double value);
    double filter(double input);
};

}