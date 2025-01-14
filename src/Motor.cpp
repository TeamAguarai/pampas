#ifdef USING_VSCODE_AS_EDITOR
    #include "Motor.h"
#endif


namespace pampas {
    
Motor* motorInstance = nullptr;

Motor::Motor() {
    motorInstance = this;
    std::signal(SIGINT, [](int) {
        motorInstance->cleanup();  
    });
}

Motor::~Motor() 
{
    this->cleanup();
}

void Motor::cleanup() 
{
    this->setPulseWidth(this->pulseWidth.steady);
}

void Motor::setPin(int pin) 
{
    this->pin = pin;
    gpio::pinMode(pin, PWM_OUTPUT);
}

void Motor::setPulseWidthRange(double min, double steady, double max) {
    this->pulseWidth.set(min, steady, max);
}

void Motor::setPulseWidth(double pulseWidth) 
{
    if (this->pulseWidth.isDefined() == false) throw std::invalid_argument( "Faltan definir los valores de ancho de pulso." );
    if (this->pin == -1) throw std::invalid_argument( "Faltan definir el pin del motor" );
    gpio::pwmWrite(this->pin, this->pulseWidth.validate(pulseWidth));
}


/* Analizar posible implementacion
void Motor::runForMilliseconds(int milliseconds, double pulseWidthMs) {
    std::thread([=]() {
        this->setPulseWidth(pulseWidthMs); 
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }).detach();
}
*/


}