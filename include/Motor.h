#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #pragma once
    #include "PulseWidth.h"
#endif

#ifdef CONTROL_LIBRARY
    #include "pampas.h"
#endif

#include <csignal>

namespace pampas {
    
class Motor
{
private:
public:
    Motor();
    ~Motor();
    PulseWidth pulseWidth;
    int pin = -1;
    void setPin(int pin);
    void setPulseWidthRange(double min, double steady, double max);
    void setPulseWidth(double pulseWidth);
    void cleanup(); // llamar si o si a este metodo al final del programa, en caso de un programa abortada ejecutara automaticamente.
    // void runForMilliseconds(int milliseconds, double speed); ! revisar posible implementacion
};
}
