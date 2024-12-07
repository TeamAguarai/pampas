#include "Vehicle.h"
#include <thread>
#include <iostream>
#include <chrono>


Vehicle* vehicleInstance = nullptr;

Vehicle::Vehicle() {
    vehicleInstance = this;
}


