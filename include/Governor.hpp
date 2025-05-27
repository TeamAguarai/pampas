/*
Governor.hpp

Controller class for managing the speed of a HYPER VS 1/8 BUGGY NITRO engine.

This class requires the following preconfigured objects, passed via constructor:
- Motor
- PID
- Velocimeter
- Conversion

Example usage (all objects must be configured as shown):

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
#include "Velocimeter.hpp"
#include "PID.hpp"
#include "AdaptivePID.hpp"
#include "Conversion.hpp"
#include "gpio.hpp"
#include "Motor.hpp"
#endif

#include <thread>
#include <iostream>
#include <functional>
#include <variant>

namespace pampas {

class Governor {
public:
    Governor(Motor &motor, PID classic_pid_controller, Velocimeter &velocimeter, Conversion &ms_to_pulsewidth_conversion);
    Governor(Motor &motor, AdaptivePID adaptive_pid_controller, Velocimeter &velocimeter, Conversion &ms_to_pulsewidth_conversion);
    void run(float speed);
    void stop();
private:
    Motor &motor_;
    Velocimeter &velocimeter_;
    Conversion &ms_to_pulsewidth_conversion_;

    PID classic_pid_controller_;
    AdaptivePID adaptive_pid_controller_;
    
    std::thread control_thread_;
    PID::Type controller_type_;

    void runAtControlledSpeed(float speed);
    bool running_;
    float running_speed_;
};

// --- Implementation ---

/* Constructor for classic PID: stores all components and initializes GPIO */
Governor::Governor(
    Motor &motor, 
    PID classic_pid_controller, 
    Velocimeter &velocimeter, 
    Conversion &ms_to_pulsewidth_conversion)
    : motor_(motor), 
      classic_pid_controller_(classic_pid_controller), 
      velocimeter_(velocimeter), 
      ms_to_pulsewidth_conversion_(ms_to_pulsewidth_conversion)
{
    gpio::setupGpioPinout();
    bool running_ = false;
    float running_speed_ = 0;
    controller_type_ = PID::Type::CLASSIC;
}

/* Constructor for adaptive PID: stores all components and initializes GPIO */
Governor::Governor(
    Motor &motor, 
    AdaptivePID adaptive_pid_controller, 
    Velocimeter &velocimeter, 
    Conversion &ms_to_pulsewidth_conversion)
    : motor_(motor), 
      adaptive_pid_controller_(adaptive_pid_controller), 
      velocimeter_(velocimeter), 
      ms_to_pulsewidth_conversion_(ms_to_pulsewidth_conversion)
{
    gpio::setupGpioPinout();
    running_ = false;
    running_speed_ = 0;
    controller_type_ = PID::Type::ADAPTIVE;
}

/* Stops the control loop and joins the thread if running */
void Governor::stop() {
    running_ = false;
    // if (control_thread_.joinable()) control_thread_.join();
}

/* Control loop that adjusts motor speed based on PID and velocity feedback */
void Governor::runAtControlledSpeed(float speed) {

    motor_.setPulseWidth(ms_to_pulsewidth_conversion_.convert(speed * 0.8)); // arranque al sistema

    velocimeter_.start();
    velocimeter_.waitForUpdate();
    
    float pid_output;
    
    while (running_) {
        velocimeter_.start();
        velocimeter_.waitForUpdate();

        if (controller_type_ == PID::Type::CLASSIC) {
            pid_output = classic_pid_controller_.calculate(speed, velocimeter_.getSpeed(), velocimeter_.getUpdateTimeInterval());
        } else {
            pid_output = adaptive_pid_controller_.calculate(speed, velocimeter_.getSpeed(), velocimeter_.getUpdateTimeInterval());
        }

        motor_.setPulseWidth(ms_to_pulsewidth_conversion_.convert(pid_output));

        std::cout << "SetPoint (" << speed << ") | "
                  << "Velocimeter (" << velocimeter_.getSpeed() << ") | "
                  << "PID (" << pid_output << ") | "
                  << "dt(" << velocimeter_.getUpdateTimeInterval() << ") | ";
                  
        if (controller_type_ == PID::Type::CLASSIC) {
            std::cout << "KP CTE(" << classic_pid_controller_.getKp() << ") | " 
                      << "KI CTE(" << classic_pid_controller_.getKi() << ") | "
                      << "KD CTE(" << classic_pid_controller_.getKd() << ")\n";
        } else {
            std::cout << "KP ADAP(" << adaptive_pid_controller_.getKp() << ") | " 
                      << "KI ADAP(" << adaptive_pid_controller_.getKi() << ") | "
                      << "KD ADAP(" << adaptive_pid_controller_.getKd() << ")\n";
        }
            
    }
}

/* Starts the control thread at the specified speed. Stops previous one if running. */
void Governor::run(float speed) {
    if (speed == running_speed_) return;

    this->stop();
    running_speed_ = speed;
    running_ = true;
    
    control_thread_ = std::thread(&Governor::runAtControlledSpeed, this, speed);
    // control_thread_.join();
}

} // namespace pampas
