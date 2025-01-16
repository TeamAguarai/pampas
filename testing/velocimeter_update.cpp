#include "Velocimeter.h"

#include <iostream>

int main() {
    pampas::Velocimeter velocimeter;
    velocimeter.setPin(17);
    velocimeter.setWheelDiameter(0.105);
    velocimeter.setAlpha(0.9);   

    while (true)
    {
        velocimeter.start();
        velocimeter.waitForUpdate();
        std::cout << "Update!" << std::endl;
    }
}