namespace pampas {
    
class PulseWidth
{
public:
    double min = -1; // -1 como valor indefinido
    double max = -1;
    double steady = -1;
    void set(double min, double steady, double max);
    bool isDefined();
    double validate(double pulseWidthMs);
};

}