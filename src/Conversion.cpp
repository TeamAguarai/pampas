#include "Conversion.h"

Conversion::Conversion(std::function<double(double)> func) : transferFuncDouble(func) {}

Conversion::Conversion(std::function<int(int)> func) : transferFuncInt(func) {}

double Conversion::convert(double input) {
    if (transferFuncDouble) {
        return transferFuncDouble(input);
    }
    throw std::runtime_error("Función de conversión para double no definida.");
}

int Conversion::convert(int input) {
    if (transferFuncInt) {
        return transferFuncInt(input);
    }
    throw std::runtime_error("Función de conversión para int no definida.");
}