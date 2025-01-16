#include <iostream>
#include <cmath> // Para pow()

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
    
    for (double i = 0; i < 8; i += 0.1)
    {
        std::cout << "X = " << i << " | Y = " <<  polinomialRegression(i) << std::endl;
    }
    

    return 0;
}
