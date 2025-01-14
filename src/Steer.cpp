#ifdef USING_VSCODE_AS_EDITOR
    #include "Steer.h"
#endif

namespace pampas {

void Steer::setPulseWidthRange(double min, double steady, double max)
{
    this->servo.setPulseWidthRange(min, steady, max);
}

void Steer::setPin(int pin)
{
    this->servo.setPin(pin);
}

void Steer::steer(double value, double proportionalConstant) // -1 < value < 1
{
    if (value > this->max || value < this->min) throw std::invalid_argument( "El valor esta fuera del rango: -1 < value < 1" );

    double pulseWidthValue = pampas::remap(value, this->min, this->max, this->servo.pulseWidth.min, this->servo.pulseWidth.max);
    
    this->servo.setPulseWidth(pulseWidthValue);
}

}