#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #include "Error.h"
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
#endif

namespace pampas {

Error::Error(const std::string& message, const std::string& file, int line) {
    std::runtime_error(formatMessage(message, file, line));
}

std::string Error::formatMessage(const std::string& message, const std::string& file, int line) {
    std::ostringstream oss;
    oss << "Error: " << message << "\n"
        << "Archivo: " << file << "\n"
        << "Línea: " << line;
    return oss.str();
}

}
