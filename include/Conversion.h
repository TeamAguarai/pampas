#include <functional>
#include <stdexcept>
#include <string>

namespace pampas {

class Conversion {
private:
    std::function<double(double)> transferFunc;
    bool defined = false;
public:
    bool isDefined();
    void set(std::function<double(double)> func);
    double convert(double input);
};

}
