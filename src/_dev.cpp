/*
* _dev: funcionalidad para desarrollo interno
*/

#ifdef DEV
    #include "_dev.h"
#else
    #include "pampas.h"
#endif

namespace pampas {

void hello() 
{
    std::cout << "HELLO FROM PAMPAS!" << std::endl;
}

}