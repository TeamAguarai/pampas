#include <iostream>
#include <omp.h>

int main() {

    std::cout << "Hola desde el primer hilo" << std::endl;
    #pragma omp parallel
    {
        std::cout << "Hola desde el segundo hilo " << std::endl;
    }

    return 0;
}
