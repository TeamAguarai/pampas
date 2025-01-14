#ifdef USING_VSCODE_AS_EDITOR
    #include "Motor.h"
    #include "operations.h"
#endif

#include <stdexcept>
#include <iostream>

namespace pampas {

class Steer
{
private:
    Motor servo;
    double min = -1, max = 1;
public:
    void setPulseWidthRange(double min, double steady, double max);
    void setPin(int pin);
    
    // Definir el ancho de pulso del servo motor, remapeado entre -1 y 1
    void steer(double value, double proportionalConstant = 1); 

};

}