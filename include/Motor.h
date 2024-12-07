#ifndef MOTOR_H
#define MOTOR_H
#include "PulseWidth.h"

class Motor
{
public:
    PulseWidth pulseWidth;
    int pin;
    double speed;
    void definePin(int pin);
    void definePulseWidthRange(double min, double steady, double max);
    void setSpeed(float pulseWidth);
    void runForMilliseconds(int milliseconds, double speed);
};
#endif