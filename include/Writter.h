#ifndef CSV_WRITER_H
#define CSV_WRITER_H

#include <iostream>
#include <fstream>
#include <string>

class Writter {
private:
    std::ofstream file;     
    bool is_open;           
    std::string delimiter;  

public:
    Writter(std::string filename, std::string header, std::string delim = ",");
    ~Writter();
    void write_row(int pulse_width, double speed);
    void close();
};

#endif 
