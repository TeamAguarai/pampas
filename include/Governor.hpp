/*
Governor.hpp

Clase para controlar la velocidad del motor de un HYPER VS 1/8 BUGGY NITRO.

Para poder funcionar, esta clase necesita (objetos ya configurados previamente, pasados por parametro en el constructor)
- Motor
- PID
- Velocimeter
- Conversion

Ejemplo de uso (todos los objetos del ejemplos deben de definirse de la siguiente manera):
Motor motor;
motor.setPulseWidthRange(...); 
motor.setPin(...);

PID pid_controller;
pid_controller.setGains(...);
pid_controller.setParameters(...);

Velocimeter velocimeter;
velocimeter.setPin(...);
velocimeter.setWheelDiameter(...);
velocimeter.setAlpha(...);

Conversion ms_to_pulsewidth_conversion;
ms_to_pulsewidth_conversion.set(...);

Governor governor(motor, pid_controller, velocimeter, conversion);
governor.run(SPEED);

*/

#ifdef VSCODE_INTELLISENSE_SUPPORT
#include "Motor.hpp"
#include "Velocimeter.hpp"
#include "PID.hpp"
#include "Conversion.hpp"
#include "gpio.hpp"
#endif

#include <thread>
#include <iostream>
#include <functional>

namespace pampas {


class Governor {
public:
    Governor(Motor motor, PID pid_controller, Velocimeter velocimeter, Conversion ms_to_pulsewidth_conversion);
    void run(double speed);
    void stop();
    
private:
    Motor motor_;
    PID pid_controller_;
    Velocimeter velocimeter_;
    Conversion ms_to_pulsewidth_conversion_;
    std::thread control_thread_;

    bool running_ = false;
    double running_speed_ = 0;
    void runAtControlledSpeed(double speed);
};


Governor::Governor(
    Motor motor, 
    PID pid_controller, 
    Velocimeter velocimeter, 
    Conversion ms_to_pulsewidth_conversion)
    :motor_(motor), 
    pid_controller_(pid_controller), 
    velocimeter_(velocimeter), 
    ms_to_pulsewidth_conversion_(ms_to_pulsewidth_conversion)
{
    gpio::setupGpioPinout();
}

void Governor::stop() {
    running_ = false;
    if (control_thread_.joinable()) control_thread_.join();
}

void Governor::runAtControlledSpeed(double speed) 
{
    while (this->running_) {

        velocimeter_.start();
        velocimeter_.waitForUpdate();

        double newSpeed = pid_controller_.calculate(
            speed, 
            velocimeter_.getSpeed(), 
            velocimeter_.getUpdateTimeInterval()
        );

        motor_.setPulseWidth(ms_to_pulsewidth_conversion_.convert(newSpeed));

        std::cout << "SetPoint: " << speed << " | Velocimetro: " << velocimeter_.getSpeed() << " | Velocidad Nueva: " << newSpeed << " | KP(" << pid_controller_.kp_ <<")" << " KI(" << pid_controller_.ki_ << ")" << " KD(" << pid_controller_.kd_ << ") dt(" << velocimeter_.getUpdateTimeInterval() << ")\n";
    }
}

void Governor::run(double speed) 
{
    if (speed == this->running_speed_) return; // evita que se cree un nuevo thread para una velocidad ya definida

    this->stop();
    this->running_speed_ = speed;
    this->running_ = true;
    
    this->control_thread_ = std::thread(&Governor::runAtControlledSpeed, this, speed);
}

} // // namespace pampas