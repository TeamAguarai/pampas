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

enum CONTROL {
    CLASSIC,
    ADAPTATIVE
};

class Governor {
public:
    Governor(Motor motor, std::variant<PID, AdaptivePID> pid_controller, Velocimeter velocimeter, Conversion ms_to_pulsewidth_conversion);
    void run(float speed);
    void stop();

private:
    Motor motor_;
    std::variant<PID, AdaptivePID> pid_controller_;
    Velocimeter velocimeter_;
    Conversion ms_to_pulsewidth_conversion_;
    std::thread control_thread_;

    void runAtControlledSpeed(float speed);
    bool running_;
    float running_speed_;
};

// --- Implementation ---

/* Constructor: stores all components and initializes GPIO */
Governor::Governor(
    Motor motor, 
    std::variant<PID, AdaptivePID> pid_controller, 
    Velocimeter velocimeter, 
    Conversion ms_to_pulsewidth_conversion)
    : motor_(motor), 
      pid_controller_(pid_controller), 
      velocimeter_(velocimeter), 
      ms_to_pulsewidth_conversion_(ms_to_pulsewidth_conversion)
{
    gpio::setupGpioPinout();
    bool running_ = false;
    float running_speed_ = 0;
}

/* Stops the control loop and joins the thread if running */
void Governor::stop() {
    running_ = false;
    if (control_thread_.joinable()) control_thread_.join();
}

/* Control loop that adjusts motor speed based on PID and velocity feedback */
void Governor::runAtControlledSpeed(float speed) {
    velocimeter_.start();
    velocimeter_.waitForUpdate();

    while (this->running_) {
        velocimeter_.start();
        velocimeter_.waitForUpdate();

        float newSpeed = pid_controller_.calculate(
            speed,
            velocimeter_.getSpeed(),
            velocimeter_.getUpdateTimeInterval()
        );

        motor_.setPulseWidth(ms_to_pulsewidth_conversion_.convert(newSpeed));

        std::cout << "SetPoint (" << speed << ") | "
                  << "Velocimeter (" << velocimeter_.getSpeed() << ") | "
                  << "PID (" << newSpeed << ") | "
                  << "KP(" << pid_controller_.getKp() << ") | "
                  << "KI(" << pid_controller_.getKi() << ") | "
                  << "KD(" << pid_controller_.getKd() << ") | "
                  << "dt(" << velocimeter_.getUpdateTimeInterval() << ")\n";
    }
}

/* Starts the control thread at the specified speed. Stops previous one if running. */
void Governor::run(float speed) {
    if (speed == running_speed_) return;

    this->stop();
    running_speed_ = speed;
    running_ = true;

    control_thread_ = std::thread(&Governor::runAtControlledSpeed, this, speed);
}

} // namespace pampas
