#ifndef SPEED_H
#define SPEED_H

class Speed
{
private:
    double min, max;
public:
    double validate(double speed);
    Speed(double min, double max);
    ~Speed();
};

Speed::Speed(double min, double max):min(min),max(max) {}

Speed::~Speed()
{
    
}


#endif