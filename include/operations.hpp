/*
operations.hpp

Utilidades varias.

*/

#include <algorithm>
#include <wiringPi.h>
#include <chrono>
#include <cmath>

namespace pampas {

// Referencia: https://stackoverflow.com/questions/8684327/c-map-number-ranges
template <typename T>
T remap(T value, T in_min, T in_max, T out_min, T out_max);

void delay(int ms);

// Referencia: https://stackoverflow.com/questions/70221264/how-do-i-set-the-precision-of-a-float-variable-in-c
float round(float num, int decimals);
double round(double num, int decimals);
int round(int num, int decimals); // No afecta enteros, solo los devuelve.


template <typename T>
T remap(T value, T in_min, T in_max, T out_min, T out_max) {
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

void delay(int ms) {
	::delay(ms); // wiringPi function
}

// Redondeo para float
float round(float num, int decimals) {
    float factor = std::pow(10.0f, decimals);
    return std::round(std::round(num * (factor * 10.0f)) / 10.0f) / factor;
}

// Redondeo para double
double round(double num, int decimals) {
    double factor = std::pow(10.0, decimals);
    return std::round(std::round(num * (factor * 10.0)) / 10.0) / factor;
}

// Redondeo para enteros (No afecta valores enteros)
int round(int num, int) {
    return num;
}

} 