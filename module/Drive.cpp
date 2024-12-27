#include "Drive.h"

Drive::Drive()
{
    gpio::setupGpioPinout();
}

void Drive::definePid(double kp, double ki, double kd, double minOutput, double maxOutput) {
    this->pid.define(kp, ki, kd, minOutput, maxOutput);
}

void Drive::defineTransferFunction(std::function<double(double)> func) {
    this->MsToPulseWidth.define(func);
}

void Drive::stop() {
    this->running = false;
    this->control.join();
}

void Drive::run(double speed) 
{
    if (speed == this->runningSpeed) return; // evita que se cree un nuevo thread para una velocidad ya definida

    this->stop();
    this->runningSpeed = speed;
    this->running = true;
    
    this->control = std::thread(this, [=]() {
        while (this->running) {
            this->velocimeter.start();
            this->velocimeter.waitForUpdate();
            double movementSpeed = this->pid.calculate(speed, velocimeter.getSpeed());
            std::cout << movementSpeed;
            // this->motor.setPulseWidth(this->MsToPulseWidth.convert(movementSpeed));
        }
    });
}