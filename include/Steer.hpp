/*
Steer.cpp

Clase para controlar la direccion del HYPER VS 1/8 BUGGY NITRO a traves de su servo motor.



#ifdef VSCODE_INTELLISENSE_SUPPORT
#include "Motor.hpp"
#include "operations.hpp"
#endif

#include <stdexcept>
#include <iostream>

namespace pampas {

class Steer
{
private:
    Motor servo;
    double min = -1, max = 1;
public:
    void setPulseWidthRange(double min, double steady, double max);
    void setPin(int pin);
    
    // Definir el ancho de pulso del servo motor, remapeado entre -1 y 1
    void steer(double value, double proportionalConstant = 1); 

};

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

    double pulseWidthValue = pampas::remap(value, this->min, this->max, this->servo.pulsewidth_.min, this->servo.pulseWidth.max);
    
    this->servo.setPulseWidth(pulseWidthValue);
}

}
*/
