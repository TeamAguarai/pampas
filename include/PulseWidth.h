#ifndef PULSEWIDTH_H
#define PULSEWIDTH_H

namespace control {
    
class PulseWidth
{
public:
    double min = -1; // -1 como valor indefinido
    double max = -1;
    double steady = -1;
    void define(double min, double steady, double max);
    bool isDefined();
    double validate(double pulseWidthMs);
};

}
#endif