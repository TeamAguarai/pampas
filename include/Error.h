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

#include <iostream>
#include <stdexcept> 
#include <string>
#include <sstream>   

class Error : public std::runtime_error {
public:
    Error(const std::string& message, const std::string& file, int line);

private:
    static std::string formatMessage(const std::string& message, const std::string& file, int line);
};

#define THROW_ERROR(msg) throw Error(msg, __FILE__, __LINE__)
