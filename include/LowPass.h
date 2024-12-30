#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

namespace control {
    
class LowPass {
private:
    double alpha;       // Coeficiente de suavizado
    double prevOutput;  // Última salida (x_{n-1})

public:
    LowPass(double alpha);
    double filter(double input);
};

}

#endif 
