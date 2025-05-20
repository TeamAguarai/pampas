/*
gpio.hpp

Wrapper de funciones de wiringPi para controlar los pines del RaspberryPi.

La intencion de este archivo, es de si en caso de ser necesario, es que cambiar la libreria
de control de pines (la actual es wiringPi) sea un proceso sin complicaciones.

En caso de querer cambia la libreria control de pines, esta debe ser capaz de realizar TODAS
las funciones del namespace gpio.

*/

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

}

namespace gpio {

void setupGpioPinout() {
    ::wiringPiSetupGpio();  
}

void pwmWrite(int pin, double pulseWidthMs) {

    // AGUARAI STEADY STATE
    if (pulseWidthMs > 2.0) pulseWidthMs = 2.0;
    else if (pulseWidthMs < 1.0) pulseWidthMs = 1.0;

    
    // Calculate the range (resolution) and the divisor to adjust the frequency
    double periodMs = 1000.0 / PWM_FREQUENCY; // Period in milliseconds
    int range = 1024; // PWM resolution (default in WiringPi)
    int divisor = static_cast<int>(19200000.0 / (PWM_FREQUENCY * range)); // PWM clock at 19.2 MHz

    // Set the divisor and range
    ::pwmSetMode(0);      // "Mark-Space" mode for precision
    ::pwmSetRange(range);           // Set the range for the duty cycle
    ::pwmSetClock(divisor);         // Set the divisor to adjust the frequency

    // Calculate the duty cycle corresponding to the desired pulse width
    int dutyCycle = static_cast<int>((pulseWidthMs / periodMs) * range);

    // Set the duty cycle
    ::pwmWrite(pin, dutyCycle);
}


void pinMode(int pin, int mode) {
    ::pinMode(pin, mode);
}

void digitalWrite(int pin, int value) {
    ::digitalWrite(pin, value);
}

int digitalRead(int pin) {
    return ::digitalRead(pin);
}

void onInterrupt (int pin, int edgeType,  void (*function)(void)) {
    ::wiringPiISR(pin, edgeType, function);
}

void stopOnInterrupt(int pin) {
    ::wiringPiISRStop(pin); 
}

}

} // namespace pampas