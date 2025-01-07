#if defined(PAMPAS_DEV) && defined(PAMPAS_LIBRARY)
    #error "No se puede definir PAMPAS_DEV y PAMPAS_LIBRARY al mismo tiempo."
#endif

#ifdef PAMPAS_DEV
    #pragma once
#endif

#ifdef PAMPAS_LIBRARY
    #include "pampas.h"
#endif

/*
Clase Error: creado por ChatGPT
*/

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


void throwError(std::string msg);

}

