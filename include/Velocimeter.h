#ifndef VELOCIMETER_H
#define VELOCIMETER_H

#include <time.h>
#include <iostream>
#include <chrono>

class Velocimeter
{
private:
    double wheelCircumference;
    double timeInterval;
    struct timespec startTime, endTime;
    static void pulseHandlerWrapper();
    bool started = false;
public:
    int pin;
    double speed;
    double wheelDiameter;
    Velocimeter();
    void definePin(int pin);
    void defineWheelDiameter(double wheelDiameter);
    void pulseHandler();
    void start();
};

#endif