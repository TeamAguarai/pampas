/*
PulseWidth.hpp

Clase para controlar el rango de valores del ancho de pulso de una señal PWM.

Ejemplo de uso:
PulseWidth pulsewidth;
pulsewidth.set(...);
in_range_pulsewidth = pulsewidth.validate(pulseWidthMs);

*/


namespace pampas {
    
class PulseWidth
{
public:
    float min_ = -1; 
    float max_ = -1;
    float steady_ = -1;
    void set(float min, float steady, float max);
    bool isDefined();
    float validate(float pulseWidthMs);
};


void PulseWidth::set(float min, float steady, float max) 
{ 
    min_ = min; 
    max_ = max;
    steady_ = steady; 
}

bool PulseWidth::isDefined() 
{
    if (min_ == -1 || max_ == -1 || steady_ == -1) return false;
    return true;
}

float PulseWidth::validate(float pulseWidth) 
{
    if (pulseWidth > max_) return max_;
    if (pulseWidth < min_) return min_;
    return pulseWidth;
}

}