#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #include "Drive.h"
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
#endif


namespace pampas {

Drive* driveInstance = nullptr;


Drive::Drive()
{
    pampas::gpio::setupGpioPinout();
}

void Drive::setPid(double kp, double ki, double kd, double tau, double minOutput, double maxOutput, double minOutputInt, double maxOutputInt) 
{
    this->pid.setGains(kp, ki, kd);
    this->pid.setParameters(tau, minOutput, maxOutput, minOutputInt, maxOutputInt);
}

void Drive::setVelocimeter(int pin, double wheelDiameterCM, double alpha) 
{
    this->velocimeter.setPin(pin);
    this->velocimeter.setWheelDiameter(wheelDiameterCM);
    this->velocimeter.setAlpha(alpha);
}


void Drive::setMotor(int pin, double pulseWidthMin, double pulseWidthSteady, double pulseWidthMax) 
{
    this->motor.setPin(pin);
    this->motor.setPulseWidthRange(pulseWidthMin, pulseWidthSteady, pulseWidthMax);
}

void Drive::setTransferFunction(std::function<double(double)> func) 
{
    this->MsToPulseWidth.set(func);
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
        double newSpeed = this->pid.calculate(
            speed, 
            this->velocimeter.getSpeed(), 
            this->velocimeter.getUpdateTimeInterval()
        );

        this->motor.setPulseWidth(this->MsToPulseWidth.convert(newSpeed));
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