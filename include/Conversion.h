#ifndef CONVERSION_H
#define CONVERSION_H

#include <functional>
#include <stdexcept>
#include <string>

class Conversion {
private:
    std::function<double(double)> transferFunc;
    bool defined = false;
public:
    bool isDefined();
    void define(std::function<double(double)> func);
    double convert(double input);
};

#endif