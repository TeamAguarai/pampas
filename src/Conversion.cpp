#ifdef DEV
    #include "Conversion.h"
#else
    #include "control.h"
#endif


namespace control {

bool Conversion::isDefined() 
{
    return this->defined;
}

void Conversion::define(std::function<double(double)> func) 
{
    this->defined = true;
    this->transferFunc = func;
    
}

double Conversion::convert(double input) 
{
    if (!this->defined) throw std::runtime_error("Función de conversión no definida.");
    return this->transferFunc(input);
}

}

