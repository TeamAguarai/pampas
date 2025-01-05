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

void Steer::steer(double value)
{
    if (!this->servo.pulseWidth.isDefined()) throw std::invalid_argument( "Faltan definir el rango de ancho de pulso de Steer" );

    double pulseWidthValue = pampas::clip(value, this->servo.pulseWidth.min, this->servo.pulseWidth.max);
    
    std::cout << "PW STEER:" << pulseWidthValue << std::endl; // for debug
    
    this->servo.setPulseWidth(pulseWidthValue);
}

}