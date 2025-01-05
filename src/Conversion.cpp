#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #include "Conversion.h"
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
#endif



namespace pampas {

bool Conversion::isDefined() 
{
    return this->defined;
}

void Conversion::set(std::function<double(double)> func) 
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

