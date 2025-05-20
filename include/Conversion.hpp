/*
Conversion.hpp

Clase para gestionar las conversion de valores Ancho de pulso (PWM - nano segundos) a Velocidad (m/s).

Ejemplo de uso:
Conversion conversion;
conversion.set(least_square_function);
data = conversion.convert(value);

ToDo: mejorar mensaje de error para funcion no definida.
*/


#include <functional>
#include <stdexcept>
#include <string>

namespace pampas {

class Conversion {
private:
    std::function<double(double)> transfer_function_;
    bool defined_ = false;
public:
    void set(std::function<double(double)> func);
    bool isDefined();
    double convert(double input);
};

bool Conversion::isDefined() 
{
    return this->defined_;
}

void Conversion::set(std::function<double(double)> func) 
{
    this->defined_ = true;
    this->transfer_function_ = func;
    
}

double Conversion::convert(double input) 
{
    if (!this->defined_) throw std::runtime_error("Función de conversión no definida.");
    return this->transfer_function_(input);
}

} // namespace pampas
