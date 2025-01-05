/*
* _dev: funcionalidad para desarrollo interno
*/

#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #include "_dev.h"
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
#endif


namespace pampas {

void hello() 
{
    std::cout << "HELLO FROM PAMPAS!" << std::endl;
}

}