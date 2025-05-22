/*
Exception.hpp

Manejo de mensajes de error.

*/

#include <iostream>
#include <stdexcept>

/* Levanta un runtime_error que incluye un mensaje y detalles del archivo y linea */
#define EXCEPTION(msg) \
    throw std::runtime_error(std::string("ERROR en ") + __FILE__ + ":" + std::to_string(__LINE__) + " → " + msg)