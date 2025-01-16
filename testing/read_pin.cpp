#include <iostream>
#include "../pampas.h"

int main(int argc, char* argv[]) {

    if (argc <= 1) {
        std::cout << "Error. Debe proporcionar un pin." << std::endl;
        return 0;
    }

    pampas::gpio::setupGpioPinout();

    int pin = std::atoi(argv[1]);
    
    long int counter = 0;
    while (true) {
        std::cout << "\rRegistro [" << counter << "] -> valor Pin(" << pin << "): " << pampas::gpio::digitalRead(pin) << std::endl;
        pampas::gpio::delay(100);
        counter++;
    }

    return 0;
}
