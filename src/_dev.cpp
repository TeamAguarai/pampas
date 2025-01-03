/*
* _dev: funcionalidad para desarrollo interno
*/

#ifdef DEV
    #include "_dev.h"
#else
    #include "control.h"
#endif

namespace control {

void hello() 
{
    std::cout << "HELLO FROM CONTROL!" << std::endl;
}

}