#include "Velocimeter.h"
#include "gpio.h"
#include "Writer.h"

Velocimeter* velocimeterInstance = nullptr;

Velocimeter::Velocimeter() 
{
    gpio::setupGpioPinout();
    velocimeterInstance = this;

    // prepara el cleanup en caso de un programa cancelado precozmente
    std::signal(SIGINT, [](int) {
        velocimeterInstance->cleanup();
    });    

}

void Velocimeter::cleanup() {
    gpio::stopOnInterrupt(velocimeterInstance->pin);
}

void Velocimeter::definePin(int pin) 
{
    this->pin = pin;
}

void Velocimeter::defineWheelDiameter(double wheelDiameter) 
{
    this->wheelDiameter = wheelDiameter;
    this->wheelCircumference = 2 * (wheelDiameter/2) * 3.1416;
}

void Velocimeter::pulseHandlerWrapper() 
{
    velocimeterInstance->pulseHandler();
}

void Velocimeter::pulseHandler() 
{
    this->distance += this->wheelCircumference;

    clock_gettime(CLOCK_MONOTONIC, &this->endTime);
    this->timeInterval = (endTime.tv_sec - startTime.tv_sec) +
                   (endTime.tv_nsec - startTime.tv_nsec) / 1e9;
    this->speed = (timeInterval > 0) ? (this->wheelCircumference / this->timeInterval) : 0.0;
    clock_gettime(CLOCK_MONOTONIC, &this->startTime);
    
    this->udpated = true;    
}
 
void Velocimeter::start() 
{
    this->udpated = false;
    if (this->started == true) return;

    gpio::pinMode(this->pin, gpio::INPUT);
    gpio::onInterrupt(this->pin, gpio::INT_EDGE_RISING, &pulseHandlerWrapper);
    clock_gettime(CLOCK_MONOTONIC, &this->startTime);
    
    this->started = true;
}

double Velocimeter::getSpeed() 
{
    return this->speed;
}

double Velocimeter::getDistance() 
{
    return this->distance;
}

void Velocimeter::resetDistance() 
{
    this->distance = 0;
}

void Velocimeter::waitForUpdate(double timeoutSeconds) {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end;
    double timeDifference = 0;
    while (!this->udpated && timeDifference <= timeoutSeconds) {
        end = std::chrono::steady_clock::now();
        timeDifference = std::chrono::duration_cast<std::chrono::seconds>(end - begin).count();
    }

    if (timeDifference >= timeoutSeconds) {
        this->speed = 0; // cuando se cumple el timeout, la velocidad se supone que es nula
    }
}
