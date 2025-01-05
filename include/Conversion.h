#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #pragma once
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
#endif



#include <functional>
#include <stdexcept>
#include <string>

namespace pampas {

class Conversion {
private:
    std::function<double(double)> transferFunc;
    bool defined = false;
public:
    bool isDefined();
    void set(std::function<double(double)> func);
    double convert(double input);
};

}
