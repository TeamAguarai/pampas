/*
Motor.hpp

Clase para accionar el motor del HYPER VS 1/8 BUGGY NITRO.

Ejemplo de uso:
Motor motor;
motor.setPin(...);
motor.setPulseWidthRange(...);
motor.setPulseWidth(value);

*/

#ifdef VSCODE_INTELLISENSE_SUPPORT
#include "PulseWidth.hpp"
#include "gpio.hpp"
#endif
    
#include <csignal>
#include <stdexcept>

namespace pampas {
    
class Motor
{
public:
    Motor();
    ~Motor();
    void setPin(int pin);
    void setPulseWidthRange(double min, double steady, double max);
    void setPulseWidth(double pulseWidth);
    void cleanup();
    PulseWidth pulseWidth_;

private:
    int pin = -1;
};


Motor::Motor() {}

void Motor::cleanup() 
{
    this->setPulseWidth(this->pulseWidth_.steady);
}

void Motor::setPin(int pin) 
{
    this->pin = pin;
    gpio::pinMode(pin, PWM_OUTPUT); // PWM_OUTPUT is a wiringPi constant
}

void Motor::setPulseWidthRange(double min, double steady, double max) {
    this->pulseWidth_.set(min, steady, max);
}

void Motor::setPulseWidth(double pulseWidth) 
{
    if (this->pulseWidth_.isDefined() == false) throw std::invalid_argument( "Faltan definir los valores de ancho de pulso." );
    if (this->pin == -1) throw std::invalid_argument( "Faltan definir el pin del motor" );

    gpio::pwmWrite(this->pin, this->pulseWidth_.validate(pulseWidth));
}

} // namespace pampas