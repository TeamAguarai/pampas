#ifndef WRITER_H
#define WRITER_H

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