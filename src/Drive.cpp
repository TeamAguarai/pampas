#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #include "Drive.h"
#endif

#ifdef CONTROL_LIBRARY
    #include "control.h"
#endif


namespace control {

Drive* driveInstance = nullptr;


Drive::Drive()
{
    control::gpio::setupGpioPinout();
}

void Drive::definePid(double kp, double ki, double kd, double tau, double minOutput, double maxOutput, double minOutputInt, double maxOutputInt) 
{
    this->pid.setGains(kp, ki, kd);
    this->pid.setParameters(tau, minOutput, maxOutput, minOutputInt, maxOutputInt);
}

void Drive::defineVelocimeter(int pin, double wheelDiameterCM, double alpha) 
{
    this->velocimeter.definePin(pin);
    this->velocimeter.defineWheelDiameter(wheelDiameterCM);
    this->velocimeter.defineAlpha(alpha);
}


void Drive::defineMotor(int pin, double pulseWidthMin, double pulseWidthSteady, double pulseWidthMax) 
{
    this->motor.definePin(pin);
    this->motor.definePulseWidthRange(pulseWidthMin, pulseWidthSteady, pulseWidthMax);
}

void Drive::defineTransferFunction(std::function<double(double)> func) 
{
    this->MsToPulseWidth.define(func);
}

void Drive::stop() {
    this->running = false;
    if (this->control.joinable()) this->control.join();
}



void Drive::controlledSpeed(double speed) 
{
    while (this->running) {

        this->velocimeter.start();
        this->velocimeter.waitForUpdate();

        // Calcular velocidad utilizando PID
        double movementSpeed = this->pid.calculate(
            speed, 
            this->velocimeter.getSpeed(), 
            this->velocimeter.getUpdateTimeInterval()
        );

        this->motor.setPulseWidth(this->MsToPulseWidth.convert(movementSpeed));
    }
}

void Drive::run(double speed) 
{
    if (speed == this->runningSpeed) return; // evita que se cree un nuevo thread para una velocidad ya definida

    this->stop();
    this->runningSpeed = speed;
    this->running = true;
    
    this->control = std::thread(&Drive::controlledSpeed, this, speed);
    

}

}