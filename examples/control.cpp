#include "../control.h"
#include <iostream>
#include <cmath> 

double polinomialRegression(double speedMS) {
    return 0.0000 * pow(speedMS, 8) -
        0.0002 * pow(speedMS, 7) +
        0.0019 * pow(speedMS, 6) -
        0.0059 * pow(speedMS, 5) -
        0.0031 * pow(speedMS, 4) +
        0.0658 * pow(speedMS, 3) -
        0.1496 * pow(speedMS, 2) +
        0.1291 * speedMS +
        1.5348;
}


int main() {
    std::cout << "Inicio" << std::endl;

    std::cout << "PR: " << polinomialRegression(3) << std::endl;

    control::Drive drive;
    drive.definePid(0.4369, 0.6735, 0, 0, 0, 7.4, -10, 10);
    drive.defineTransferFunction(polinomialRegression);
    drive.defineMotor(13, 1.57, 1.57, 2.0);
    drive.defineVelocimeter(17, 0.105, 0.1);
    
    while (true) 
    {
        drive.run(2.5);
    }

    return 0;
}