#include "operations.h"

namespace control
{
// ToDo: cambiar a Template
double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}
    
}
