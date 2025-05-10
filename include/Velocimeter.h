#ifdef USING_VSCODE_AS_EDITOR
    #include "LowPass.h"
    #include "gpio.h"
    #include "operations.h"
#endif

#include <time.h>
#include <functional>
#include <csignal>
#include <chrono>
#include <stdexcept>

namespace pampas {   

class Velocimeter
{
private:
    double wheelCircumference;
    double timeInterval = 0.0;
    static void pulseHandlerWrapper();
    bool distance = 0.0;
    int pin;
    double speed = 0;
    double wheelDiameter;
    bool started = false;
    bool udpated = false;
    void pulseHandler();
    LowPass<double> filter;
public:
    struct timespec startTime = {0}, endTime = {0};
    Velocimeter();
    ~Velocimeter();
    
    void setPin(int pin);
    void setWheelDiameter(double wheelDiameter);
    void setAlpha(double alpha);

    double getUpdateTimeInterval();
    double getSpeed();
    double getDistance();
    void resetDistance();

    void waitForUpdate(double timeoutSeconds = 2.0f);
    void start(); // este metodo debe llamarse al iniciar las mediciones de velocidad
    void cleanup(); // llamar si o si a este metodo al final del programa, en caso de un programa abortado ejecutara automaticamente.
};

}