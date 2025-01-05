#include "operations.h"

namespace pampas
{
// ToDo: cambiar a Template
double clip(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}
    
}
