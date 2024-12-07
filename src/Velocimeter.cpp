#include "Velocimeter.h"
#include "gpio.h"
#include <iostream>
#include <chrono>

Velocimeter* velocimeterInstance = nullptr;

Velocimeter::Velocimeter() {
    velocimeterInstance = this;
}

void Velocimeter::definePin(int pin) 
{
    this->pin = pin;
}

void Velocimeter::defineWheelDiameter(double wheelDiameter) {
    this->wheelDiameter = wheelDiameter;
    this->wheelCircumference = 2 * (wheelDiameter/2) * 3.1416;
}

void Velocimeter::pulseHandlerWrapper() {
    velocimeterInstance->pulseHandler();
}

void Velocimeter::pulseHandler() {
    clock_gettime(CLOCK_MONOTONIC, &this->endTime);
    this->timeInterval = (endTime.tv_sec - startTime.tv_sec) +
                   (endTime.tv_nsec - startTime.tv_nsec) / 1e9;

    this->speed = (timeInterval > 0) ? (this->wheelCircumference / this->timeInterval) : 0.0;
    // std::cout << this->speed  << std::endl;
    clock_gettime(CLOCK_MONOTONIC, &startTime);
}
 
void Velocimeter::start() {
    if (this->started == true) return;
    gpio::pinMode(this->pin, gpio::INPUT);

    gpio::onInterrupt(this->pin, gpio::INT_EDGE_RISING, &pulseHandlerWrapper);
    clock_gettime(CLOCK_MONOTONIC, &this->startTime);
    this->started = true;
}
