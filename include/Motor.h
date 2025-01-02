#ifndef MOTOR_H
#define MOTOR_H

#include <csignal>

#ifdef DEV
    #include "PulseWidth.h"
#else
    #include "control.h"
#endif

namespace control {
    
class Motor
{
private:
public:
    Motor();
    ~Motor();
    PulseWidth pulseWidth;
    int pin = -1;
    void definePin(int pin);
    void definePulseWidthRange(double min, double steady, double max);
    void setPulseWidth(double pulseWidth);
    void cleanup(); // llamar si o si a este metodo al final del programa, en caso de un programa abortada ejecutara automaticamente.
    // void runForMilliseconds(int milliseconds, double speed); ! revisar posible implementacion
};
}

#endif