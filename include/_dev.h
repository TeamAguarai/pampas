/*
* _dev: funcionalidad para desarrollo interno
*/

#include <iostream>

#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #pragma once
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
#endif

namespace pampas
{
void hello();
}
