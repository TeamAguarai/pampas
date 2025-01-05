#include "../include/Drive.h"
#include <iostream>
#include <cmath> 

#include <cmath>

double polinomialRegression(double x) {
    return 1.5406368187143187 
        + (0.2630704132809448) * pow(x, 1)
        + (-0.7839709920881821) * pow(x, 2)
        + (1.1329846648132278) * pow(x, 3)
        + (-0.9313254302627445) * pow(x, 4)
        + (0.4795173149457668) * pow(x, 5)
        + (-0.1621289088299355) * pow(x, 6)
        + (0.0365891255894518) * pow(x, 7)
        + (-0.0054579880129316) * pow(x, 8)
        + (0.0005162340433127) * pow(x, 9)
        + (-0.0000280155501921) * pow(x, 10)
        + (0.0000006638052154) * pow(x, 11);
}




int main() {
    std::cout << "Inicio" << std::endl;

    std::cout << "PR: " << polinomialRegression(3) << std::endl;

    pampas::Drive drive;
    drive.setPid(0.4369, 0.6735, 0, 0, 0, 7.4, -10, 10);
    drive.setTransferFunction(polinomialRegression);
    drive.setMotor(13, 1.57, 1.57, 2.0);
    drive.setVelocimeter(17, 0.105, 0.1);
    
    while (true) 
    {
        drive.run(2.5);
    }

    return 0;
}