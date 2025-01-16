#include "Steer.h"

int main()
{
    pampas::Steer steering;
    steering.setPulseWidthRange(1200, 1500, 1800);
    steering.steer(0.0);
    steering.steer(0.5);
    steering.steer(0.8);
    steering.steer(1.0);
    steering.steer(-0.5);
    steering.steer(-0.8);
    steering.steer(-1.0);
    
}
