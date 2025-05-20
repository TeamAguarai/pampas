/*
LowPass.hpp

Clase para ejecutar un filtro de paso bajo.

Ejemplo de uso:
LowPass low_pass_filter;
low_pass_filter.setInitialValue(...);
low_pass_filter.setAlpha(...);
low_pass_filter.filter(value);

*/


namespace pampas {

template<typename T>
class LowPass {
private:
    T alpha;       // Coeficiente de suavizado (debe ser inicializado con setAlpha)
    T prevOutput;  
    bool alphaDefined = false;
public:
    LowPass();  
    void setInitialValue(T value);
    void setAlpha(T value);
    T filter(T input);
};

template<typename T>
LowPass<T>::LowPass() : prevOutput(static_cast<T>(0)) {} // define el tipo de dato que se usara

template<typename T>
void LowPass<T>::setAlpha(T value) {
    if (value < static_cast<T>(0) || value > static_cast<T>(1)) {
        throw std::invalid_argument("Coeficiente de suavizado debe estar en el rango [0,1]");
    }
    this->alphaDefined = true;
    alpha = value;
}

template<typename T>
void LowPass<T>::setInitialValue(T value) {
    this->prevOutput = value;
}

template<typename T>
T LowPass<T>::filter(T input) {
    if (!this->alphaDefined) {
        throw std::runtime_error("Coeficiente de suavizado no ha sido inicializado o tiene un valor inválido.");
    }

    T output = alpha * input + (static_cast<T>(1) - alpha) * prevOutput;
    prevOutput = output;
    return output;
}

// Instanciaciones explícitas (necesario si se usa fuera del .h)

template class LowPass<double>;
template class LowPass<float>;
template class LowPass<int>;

} // namespace pampas

