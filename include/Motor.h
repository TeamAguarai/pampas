#ifdef USING_VSCODE_AS_EDITOR
    #include "PulseWidth.h"
    #include "gpio.h"
    #include <stdexcept>
#endif

#include <csignal>

namespace pampas {
    
class Motor
{
private:
public:
    Motor();
    ~Motor();
    PulseWidth pulseWidth;
    int pin = -1;
    void setPin(int pin);
    void setPulseWidthRange(double min, double steady, double max);
    void setPulseWidth(double pulseWidth);
    void cleanup(); // llamar si o si a este metodo al final del programa, en caso de un programa abortada ejecutara automaticamente.
    // void runForMilliseconds(int milliseconds, double speed); ! revisar posible implementacion
};
}
