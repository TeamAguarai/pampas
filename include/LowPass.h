#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #pragma once
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
#endif

/*
    Clase LowPass: creado por ChatGPT
*/

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