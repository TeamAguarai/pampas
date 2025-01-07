#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #include "../include/Exception.h"
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
#endif


namespace pampas {

std::string Exception::formatMessage(const std::string& message, const std::string& file, int line) {
    std::ostringstream oss;
    oss << "\nError: " << message << "\n"
        << "Archivo: " << file << "\n"
        << "Linea: " << line;
    return oss.str();
}

void throwError(std::string msg) {
    throw Exception(msg, __FILE__, __LINE__);
}

}
