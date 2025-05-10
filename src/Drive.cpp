#ifdef USING_VSCODE_AS_EDITOR
    #include "Drive.h"
#endif
#include <iostream>


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


double Drive::update_model_reference(double v_ref, double dt, double w_n, double z) {

    // Constantes internas (pueden ser ajustadas según tu sistema)
    double wn = w_n;
    double zeta = z;

    // Ecuación del modelo de referencia (sistema de segundo orden)
    double ddv_m = (wn * wn) * v_ref - 2.0 * zeta * wn * dv_m_ - (wn * wn) * v_m_;
    // std::cout << (wn * wn) * v_ref << " - ";
    // std::cout << 2.0 * zeta * wn * dv_m << " - ";
    // std::cout << (wn * wn) * v_m<< "\n\n";

    dv_m_ += ddv_m * dt;
    v_m_ += dv_m_ * dt;

    return v_m_;
}

void Drive::controlledSpeed(double speed, double w_n, double z) 
{
    
    double kp_adap = 0.0;
    double ki_adap = 0.0;
    double kd_adap = 0.0;

    double err_int = 0.0;

    double prev_error = 0.0;
    
    double g_p = 0.0;
    double g_d = 0.0;
    double g_i = 0.0;
    
    std::cout << "Ingrese el valor de gamma_p: ";
    std::cin >> g_p;
    
    std::cout << "Ingrese el valor de gamma_i: ";
    std::cin >> g_i;

    std::cout << "Ingrese el valor de gamma_d: ";
    std::cin >> g_d;

    std::cout << "Ingrese el valor de tau: ";
    double tau_filter;                      // constante de tiempo del filtro
    std::cin >> tau_filter;
    
    std::cout << "PID ADAPTATIVO: setpoint(" << speed << ")  wn(" << w_n << ") zeta(" << z << ") g_p(" << g_p << ") g_i(" << g_i << ")  g_d(" << g_d << ")\n";
    v_m_ = 0.0;
    dv_m_ = 0.0;
    
    this->pid.reset();
    
    this->motor.setPulseWidth(this->MsToPulseWidth.convert(speed));
    
    clock_gettime(CLOCK_MONOTONIC, &this->velocimeter.startTime);
    
    
    auto inicio = std::chrono::steady_clock::now();
    


    double deriv_error_filtered = 0; 

    while (true) {
        this->velocimeter.start();
        this->velocimeter.waitForUpdate(); // el programa se detiene hasta que el velocimetro detecte movimiento
        
        double measurement = this->velocimeter.getSpeed();
        double dt = this->velocimeter.getUpdateTimeInterval();

        double model_reference_setpoint = update_model_reference(speed, dt, w_n, z);
        double error = model_reference_setpoint - measurement;

        /* Kp */
        kp_adap += ((-g_p) * (0.0313*error + 0.0553*(error - prev_error)/dt) * error) * dt;
        if (kp_adap > 100) kp_adap = 100.0;
        if (kp_adap < 0) kp_adap = 0;


        /* Ki */
        err_int += error * dt;
        ki_adap += (-g_i) * (0.0313*error + 0.0553*(error - prev_error)/dt) * err_int;
        

        /* Kd y filtro */
        // Derivada cruda
        double raw_deriv = (error - prev_error) / dt;
        // Filtro exponencial de primer orden
        deriv_error_filtered = (tau_filter * deriv_error_filtered + dt * raw_deriv) / (tau_filter + dt);
        kd_adap += ((-g_d) * (0.0313*error + 0.0553*(deriv_error_filtered)/dt) * (deriv_error_filtered)/dt) * dt;
        if (kd_adap > 50) kd_adap = 50.0;
        // if (kd_adap < 0) kd_adap = 0;


        /* Define las ganacias adaptadas en un controlador PID clasico */
        this->pid.setGains(kp_adap, ki_adap, kd_adap);

        prev_error = error;
        
        /* Salida del controlador PID */
        double newSpeed = this->pid.calculate(
            model_reference_setpoint, 
            measurement, 
            dt
        );

        /* Accionar al motor al ancho de pulso adecuado para velocidad controlada */
        this->motor.setPulseWidth(this->MsToPulseWidth.convert(newSpeed));

        /* Imprime valores para debug */
        std::cout << "SetPoint: " << speed << " | MD SetPoint: " << model_reference_setpoint << " | Velocimetro: " << measurement << " | Velocidad Nueva: " << newSpeed << " | KP(" << this->pid.kp_ <<")" << " KI(" << this->pid.ki_ << ")" << " KD(" << this->pid.kd_ << ") dt(" << dt << ")\n";
    }

}

void Drive::run(double speed, double w_n, double z) 
{
    // if (speed == this->runningSpeed) return; // evita que se cree un nuevo thread para una velocidad ya definida

    // this->stop();
    this->runningSpeed = speed;
    this->running = true;
    
    this->controlledSpeed(speed, w_n, z);
    // this->control = std::thread(&Drive::controlledSpeed, this, speed, w_n, z);
}

}