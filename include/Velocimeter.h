#ifndef VELOCIMETER_H
#define VELOCIMETER_H

#include <time.h>
#include <iostream>
#include <functional>
#include <csignal>
#include <chrono>

class Velocimeter
{
private:
    double wheelCircumference;
    double timeInterval;
    struct timespec startTime, endTime;
    static void pulseHandlerWrapper();
    bool started = false;
    std::function<void(double speed)> updateCallback; // funcion que se llama cuando el velocimetro detecta un pulso
    bool distance = 0.0;
    int pin;
    double speed = 0;
    double wheelDiameter;
    bool udpated = false;
public:
    Velocimeter();
    ~Velocimeter();
    void waitForUpdate(double timeoutSeconds = 6.0);
    void definePin(int pin);
    void defineWheelDiameter(double wheelDiameter);
    void pulseHandler();
    void start(); // este metodo debe llamarse al iniciar las mediciones de velocidad
    double getSpeed();
    double getDistance();
    void resetDistance();
    void cleanup(); // llamar si o si a este metodo al final del programa, en caso de un programa abortada ejecutara automaticamente.
};

#endif