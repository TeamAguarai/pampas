/*
* _dev: funcionalidad para desarrollo interno
*/

#include <iostream>

#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #pragma once
#endif

#ifdef CONTROL_LIBRARY
    #include "pampas.h"
#endif

namespace pampas
{
void hello();
}
