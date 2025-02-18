
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

} // namespace pampas

