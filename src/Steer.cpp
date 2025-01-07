#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #include "Steer.h"
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
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
    
    std::cout << "PW STEER:" << pulseWidthValue * proportionalConstant << std::endl; // for debug
    
    // this->servo.setPulseWidth(pulseWidthValue);
}

}