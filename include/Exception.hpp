/*
Exception.hpp

Basic error handling utility.
Throws a runtime_error with a custom message including file and line information.
*/

#include <iostream>
#include <stdexcept>

/* Throws a std::runtime_error including the file name, line number, and a custom message */
#define EXCEPTION(msg) \
    throw std::runtime_error(std::string("ERROR in ") + __FILE__ + ":" + std::to_string(__LINE__) + " → " + msg)
