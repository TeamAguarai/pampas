#include <time.h>
#include <functional>
#include <csignal>
#include <chrono>
#include <stdexcept>

#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #pragma once
    #include "LowPass.h"
    #include "gpio.h"
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
#endif


namespace pampas {   

class Velocimeter
{
private:
    double wheelCircumference;
    double timeInterval;
    struct timespec startTime, endTime;
    static void pulseHandlerWrapper();
    bool distance = 0.0;
    int pin;
    double speed = 0;
    double wheelDiameter;
    bool started = false;
    bool udpated = false;
    void pulseHandler();
    LowPass filter;
public:
    Velocimeter();
    ~Velocimeter();
    
    void setPin(int pin);
    void setWheelDiameter(double wheelDiameter);
    void setAlpha(double alpha);

    double getUpdateTimeInterval();
    double getSpeed();
    double getDistance();
    void resetDistance();

    void waitForUpdate(double timeoutSeconds = 6.0);
    void start(); // este metodo debe llamarse al iniciar las mediciones de velocidad
    void cleanup(); // llamar si o si a este metodo al final del programa, en caso de un programa abortado ejecutara automaticamente.
};

}