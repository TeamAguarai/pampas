#ifndef CONVERSION_H
#define CONVERSION_H

#include <functional>
#include <stdexcept>
#include <string>

class Conversion {
private:
    std::function<double(double)> transferFuncDouble;
    std::function<int(int)> transferFuncInt; 

public:
    Conversion(std::function<double(double)> func);
    Conversion(std::function<int(int)> func);

    double convert(double input);
    int convert(int input);
};

#endif