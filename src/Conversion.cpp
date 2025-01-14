#ifdef USING_VSCODE_AS_EDITOR
    #include "Conversion.h"
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

