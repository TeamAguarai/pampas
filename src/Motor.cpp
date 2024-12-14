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

void Motor::setPulseWidth(double pulseWidth) 
{
    if (this->pulseWidth.isDefined() == false) throw std::invalid_argument( "Faltan definir los valores de ancho de pulso." );
    gpio::pwmWrite(this->pin, this->pulseWidth.validate(pulseWidth));
    this->speed = pulseWidth;
}

void Motor::runForMilliseconds(int milliseconds, double pulseWidthMs) {
    double _speed = this->speed;
    std::thread([=]() {
        this->setPulseWidth(pulseWidthMs); 
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
        this->setPulseWidth(_speed); 
    }).detach();
}

