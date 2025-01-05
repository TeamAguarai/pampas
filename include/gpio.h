#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #pragma once
#endif

#ifdef CONTROL_LIBRARY
    #include "pampas.h"
#endif

#include <wiringPi.h>

namespace pampas {
    
namespace gpio {
    void setupGpioPinout();
    void pwmWrite(int pin, double pulseWidthMs);
    void pinMode(int pin, int mode);
    void digitalWrite(int pin, int value);
    int digitalRead(int pin);
    void delay(int ms);
    void onInterrupt (int pin, int edgeType,  void (*function)(void));
    void stopOnInterrupt(int pin);
    void reset();

    /* 
    (depreceated) Se usaran en cambio las definiciones de WiringPi
    const int INPUT = 0;   
    const int OUTPUT = 1; 
    const int HIGH = 1;     
    const int LOW = 0;       
    const int PUD_OFF = 0;
    const int PUD_DOWN = 1;
    const int PUD_UP = 2;
    const int PWM_OUTPUT = 2;
    const int GPIO_CLOCK = 3;
    const int PWM_MODE_MS = 0;
    const int INT_EDGE_RISING = 2;
    */

}

}
