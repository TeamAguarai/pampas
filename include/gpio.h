#define PWM_FREQUENCY 76.92
#include <wiringPi.h>

namespace pampas {
    
namespace gpio {
    void setupGpioPinout();
    void pwmWrite(int pin, double pulseWidthMs);
    void pinMode(int pin, int mode);
    void digitalWrite(int pin, int value);
    int digitalRead(int pin);
    void onInterrupt (int pin, int edgeType,  void (*function)(void));
    void stopOnInterrupt(int pin);
    void reset();

    /* 
    (depreceated) Se usaran en cambio las definiciones de WiringPi
    const int INPUT = 0;   
    const int OUTPUT = 1; 
    const int HIGH = 1;     
    const int LOW = 0;       
    const int PUD_OFF = 0;
    const int PUD_DOWN = 1;
    const int PUD_UP = 2;
    const int PWM_OUTPUT = 2;
    const int GPIO_CLOCK = 3;
    const int PWM_MODE_MS = 0;
    const int INT_EDGE_RISING = 2;
    */

}

}
