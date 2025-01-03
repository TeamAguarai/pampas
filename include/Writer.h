#if defined(CONTROL_DEV) && defined(CONTROL_LIBRARY)
    #error "No se puede definir CONTROL_DEV y CONTROL_LIBRARY al mismo tiempo."
#endif

#ifdef CONTROL_DEV
    #pragma once
#endif

#ifdef CONTROL_LIBRARY
    #include "control.h"
#endif


#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <sstream>


namespace control { 

class Writer {
private:
    std::ofstream file;     
    bool is_open;           
    std::string delimiter;  
public:
    Writer(std::string filename, std::string header, std::string delim = ",");
    ~Writer();
    void write_row(const std::vector<std::string>& data);
    void close();
};

}

#endif