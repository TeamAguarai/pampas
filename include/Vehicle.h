#include "Motor.h"

class Vehicle {
private:
    void pulseHandler();
    void pulseHandlerWrapper();
public:
    int pulseCount = 0;
    Motor driveMotor;
    Vehicle();
    void runForMilliseconds(int milliseconds, double pulseWidthMs);
};