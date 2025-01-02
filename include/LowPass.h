#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

namespace control {
    
class LowPass {
private:
    double alpha;       // Coeficiente de suavizado
    double prevOutput = 0.0;  // Última salida (x_{n-1})

public:
    void defineAlpha(double value);
    double filter(double input);
};

}

#endif 