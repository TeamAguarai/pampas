#include <algorithm>
#include <wiringPi.h>

namespace pampas
{

/*
    Referencia: https://stackoverflow.com/questions/8684327/c-map-number-ranges
*/
template <typename T>
T remap(T value, T in_min, T in_max, T out_min, T out_max);

void delay(int ms);
} 
