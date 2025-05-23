/*
* dev.hpp: funcionalidad para desarrollo interno
*/

#include <iostream>

namespace pampas
{
void hello();

float askInput(const std::string& label) {
    float value;
    std::cout << label;
    std::cin >> value;
    return value;
}

}
