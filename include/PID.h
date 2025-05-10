/* Creditos: https://github.com/pms67/PID/blob/master/PID.h */

#include <stdexcept>

namespace pampas {    


class PID {
private:
   
    double tau_;    

    double min_output_;
    double max_output_;

    double min_output_int_;
    double max_output_int_;

    double sample_time_;

    double integrator_ = 0;
    double prev_error_ = 0;
    double differentiator_ = 0;
    double prev_measurement_ = 0;

    double out_;

    bool gains_defined_ = false;
    bool params_defined_ = false;

public:
    double kp_;     
    double ki_;     
    double kd_;  

    PID();
    void setGains(double kp, double ki, double kd);
    void reset();
    void setParameters(double tau, double min_output, double max_output, double min_output_int, double max_output_int);
    double calculate(double setpoint, double measurement, double sample_time);
};

}