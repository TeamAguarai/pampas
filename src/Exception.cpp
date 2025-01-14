#ifdef USING_VSCODE_AS_EDITOR
    #include "Exception.h"
#endif


namespace pampas {

std::string Exception::formatMessage(const std::string& message, const std::string& file, int line) {
    std::ostringstream oss;
    oss << "\nError: " << message << "\n"
        << "Archivo: " << file << "\n"
        << "Linea: " << line;
    return oss.str();
}

void raiseError(std::string msg) {
    throw Exception(msg, __FILE__, __LINE__);
}

}
