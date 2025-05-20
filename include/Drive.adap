#ifdef USING_VSCODE_AS_EDITOR
    #include "Motor.h"
    #include "Velocimeter.h"
    #include "PID.h"
    #include "Conversion.h"
    #include "gpio.h"
#endif

#include <thread>
#include <functional>

namespace pampas {

class Drive {
private:
    std::thread control;
    pampas::PID pid; // el metodo pid.set debe de llamarse antes de mover al motor
    bool running = false;
    double runningSpeed = 0;
    void controlledSpeed(double speed, double w_n, double z);
    double update_model_reference(double v_ref, double dt, double w_n, double z);
    double v_m_ = 0.0;
    double dv_m_ = 0.0;

public:
    pampas::Conversion MsToPulseWidth; 
    pampas::Motor motor;
    pampas::Velocimeter velocimeter;
    Drive();
    void run(double speed, double w_n, double z);
    void stop();

    /* Llamar estos metodos antes de mover al motor */
    void setPid(double kp, double ki, double kd, double tau, double minOutput, double maxOutput, double minOutputInt, double maxOutputInt);
    void setTransferFunction(std::function<double(double)> func);
    void setMotor(int pin, double pulseWidthMin, double pulseWidthSteady, double pulseWidthMax);
    void setVelocimeter(int pin, double wheelDiameter, double alpha);
};

}