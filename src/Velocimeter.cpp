#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #include "Velocimeter.h"
#endif

#ifdef CONTROL_LIBRARY
    #include "control.h"
#endif

namespace control {

Velocimeter* velocimeterInstance = nullptr;

Velocimeter::Velocimeter() 
{
    control::gpio::setupGpioPinout();
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

    gpio::pinMode(this->pin, INPUT);
    gpio::onInterrupt(this->pin, INT_EDGE_RISING, &pulseHandlerWrapper);
    clock_gettime(CLOCK_MONOTONIC, &this->startTime);
    
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
        gpio::delay(10); // para evitar sobrecarga
    }

    if (timeDifference >= timeoutSeconds) this->speed = 0; // cuando se cumple el timeout, la velocidad se supone que es nula
}

}