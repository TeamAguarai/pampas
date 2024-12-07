#include <stdexcept>
#include <thread>
#include <chrono>
#include "PulseWidth.h"
#include "Motor.h"
#include "gpio.h"

void Motor::definePin(int pin) 
{
    this->pin = pin;
    gpio::pinMode(pin, gpio::PWM_OUTPUT);
}

void Motor::definePulseWidthRange(double min, double steady, double max) {
    this->pulseWidth.define(min, steady, max);
}

void Motor::setSpeed(float pulseWidth) 
{
    if (this->pulseWidth.isDefined() == false) throw std::invalid_argument( "You must fully define motor pulse-width values." );
    gpio::pwmWrite(this->pin, this->pulseWidth.validate(pulseWidth));
    this->speed = pulseWidth;
}

void Motor::runForMilliseconds(int milliseconds, double pulseWidthMs) {
    double _speed = this->speed;
    this->setSpeed(pulseWidthMs); 
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds)); 
    this->setSpeed(_speed); 
}