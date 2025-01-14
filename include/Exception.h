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


void raiseError(std::string msg);

}

