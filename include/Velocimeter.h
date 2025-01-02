#ifndef VELOCIMETER_H
#define VELOCIMETER_H

#include <time.h>
#include <functional>
#include <csignal>
#include <chrono>
#include <stdexcept>

#ifdef DEV
    #include "LowPass.h"
#else
    #include "control.h"
#endif

namespace control {   

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
    
    void definePin(int pin);
    void defineWheelDiameter(double wheelDiameter);
    void defineAlpha(double alpha);

    double getUpdateTimeInterval();
    double getSpeed();
    double getDistance();
    void resetDistance();

    void waitForUpdate(double timeoutSeconds = 6.0);
    void start(); // este metodo debe llamarse al iniciar las mediciones de velocidad
    void cleanup(); // llamar si o si a este metodo al final del programa, en caso de un programa abortado ejecutara automaticamente.
};

}

#endif