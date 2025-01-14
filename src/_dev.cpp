/*
* _dev: funcionalidad para desarrollo interno
*/


#ifdef USING_VSCODE_AS_EDITOR
    #include "_dev.h"
#endif

namespace pampas {

void hello() 
{
    std::cout << "HELLO FROM PAMPAS!" << std::endl;
}

}