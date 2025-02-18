#ifdef USING_VSCODE_AS_EDITOR
    #include "operations.h"
#endif

namespace pampas
{

template <typename T>
T remap(T value, T in_min, T in_max, T out_min, T out_max) {
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

void delay(int ms) {
	::delay(ms); // wiringPi function
}

}
