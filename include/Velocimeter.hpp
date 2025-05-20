#ifdef USING_VSCODE_AS_EDITOR
    #include "LowPass.h"
    #include "gpio.h"
    #include "operations.h"
#endif

#include <time.h>
#include <functional>
#include <csignal>
#include <chrono>
#include <stdexcept>

namespace pampas {   

class Velocimeter
{
private:
    double wheelCircumference;
    double timeInterval = 0.0;
    static void pulseHandlerWrapper();
    bool distance = 0.0;
    int pin;
    double speed = 0;
    double wheelDiameter;
    bool started = false;
    bool udpated = false;
    void pulseHandler();
    LowPass<double> filter;
public:
    struct timespec startTime = {0}, endTime = {0};
    Velocimeter();
    ~Velocimeter();
    
    void setPin(int pin);
    void setWheelDiameter(double wheelDiameter);
    void setAlpha(double alpha);

    double getUpdateTimeInterval();
    double getSpeed();
    double getDistance();
    void resetDistance();

    void waitForUpdate(double timeoutSeconds = 2.0f);
    void start(); // este metodo debe llamarse al iniciar las mediciones de velocidad
    void cleanup(); // llamar si o si a este metodo al final del programa, en caso de un programa abortado ejecutara automaticamente.
};

Velocimeter* velocimeterInstance = nullptr;

Velocimeter::Velocimeter() 
{
    pampas::gpio::setupGpioPinout();
    velocimeterInstance = this;

    // prepara el cleanup en caso de un programa abortado !! REVISAR
    std::signal(SIGINT, [](int) {
        velocimeterInstance->cleanup();
    });    

}

Velocimeter::~Velocimeter() 
{
    this->cleanup();
}

void Velocimeter::cleanup() {
    gpio::stopOnInterrupt(velocimeterInstance->pin);
}

void Velocimeter::setPin(int pin) 
{
    this->pin = pin;
}

void Velocimeter::setWheelDiameter(double wheelDiameter) 
{
    this->wheelDiameter = wheelDiameter;
    this->wheelCircumference = 2 * (wheelDiameter/2) * 3.1416;
}

void Velocimeter::setAlpha(double value)
{
    this->filter.setAlpha(value);
}

void Velocimeter::pulseHandlerWrapper() 
{
    velocimeterInstance->pulseHandler();
}

void Velocimeter::pulseHandler() 
{
    if (this->started == false) return;
    
    // this->distance += this->wheelCircumference;

    clock_gettime(CLOCK_MONOTONIC, &this->endTime);
    this->timeInterval = (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_nsec - startTime.tv_nsec) / 1e9;
    this->speed = (timeInterval > 0) ? (this->wheelCircumference / this->timeInterval) : 0.0;
    clock_gettime(CLOCK_MONOTONIC, &this->startTime);
    
    this->udpated = true;    
}
 
void Velocimeter::start() 
{
    this->udpated = false;
    if (this->started == true) return;

    clock_gettime(CLOCK_MONOTONIC, &this->startTime);

    gpio::pinMode(this->pin, INPUT);
    gpio::onInterrupt(this->pin, INT_EDGE_RISING, &pulseHandlerWrapper);
    
    this->started = true;
}

double Velocimeter::getUpdateTimeInterval() 
{
    return this->timeInterval;
}

double Velocimeter::getSpeed() 
{
    // return this->speed;
    return this->filter.filter(this->speed); // valor filtrado
}

double Velocimeter::getDistance() 
{
    return this->distance;
}

void Velocimeter::resetDistance() 
{
    this->distance = 0;
}

void Velocimeter::waitForUpdate(double timeoutSeconds) 
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end;
    double timeDifference = 0;

    while (!this->udpated && timeDifference <= timeoutSeconds) {
        end = std::chrono::steady_clock::now();
        timeDifference = std::chrono::duration_cast<std::chrono::seconds>(end - begin).count();
        delay(10); // para evitar sobrecarga
    }

    if (timeDifference >= timeoutSeconds) {
        this->speed = 0; // cuando se cumple el timeout, la velocidad se supone que es nula
        // Como el sensor nunca detecto pulso, nunca se registra un intervalo de tiempo
        // para eso se le da un nuevo valor que es el timeOut 
        this->timeInterval = timeoutSeconds;
    }
    
}

}