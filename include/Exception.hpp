#include <string>
#include <iostream>
#include <stdexcept> 
#include <sstream>   

namespace pampas {

class Exception : public std::runtime_error {
public:
    Exception(const std::string& message, const std::string& file, int line)
    : std::runtime_error(formatMessage(message, file, line)) {}

private:
    static std::string formatMessage(const std::string& message, const std::string& file, int line);
};

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

} // namespace pampas

