#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #include "Conversion.h"
#endif

#ifdef CONTROL_LIBRARY
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

