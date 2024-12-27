#include "../aguarai.h"

int main() {
    Velocimeter velocimeter;
    velocimeter.definePin(17);
    velocimeter.defineWheelDiameter(0.105);

    while (true) {
        velocimeter.start();
        velocimeter.waitForUpdate();
        std::cout << "hola" << std::endl;
    }
    return 0;
}