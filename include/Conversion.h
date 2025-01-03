#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #pragma once
#endif

#ifdef CONTROL_LIBRARY
    #include "control.h"
#endif



#include <functional>
#include <stdexcept>
#include <string>

namespace control {

class Conversion {
private:
    std::function<double(double)> transferFunc;
    bool defined = false;
public:
    bool isDefined();
    void define(std::function<double(double)> func);
    double convert(double input);
};

}
