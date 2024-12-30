<div align="center">
<h1>Control</h1>

Controla un vehículo RC como el <a href="https://www.amazon.com/Hobao-HB-VS-C30BU-Hyper-Buggy-Engine/dp/B00MY3ROS2">HYPER VS 1/8 BUGGY NITRO</a> a través de un **Raspberry Pi 3** con *C++*
</div>

## ⚡️ Primeros pasos
> El proyecto está diseñado únicamente para un vehículo con características idénticas al [HYPER VS 1/8 BUGGY NITRO](https://www.amazon.com/Hobao-HB-VS-C30BU-Hyper-Buggy-Engine/dp/B00MY3ROS2)

En un [Raspberry Pi 3](https://www.raspberrypi.com/products/raspberry-pi-3-model-b/) instala [WiringPi](https://github.com/GrazerComputerClub/WiringPi)

```bash
sudo apt-get purge wiringpi
hash -r
git clone https://github.com/WiringPi/WiringPi.git
cd WiringPi
./build
```

## 🔎 Ejemplos rápidos

Utiliza un controlador PID para gestionar el movimiento del motor
```c
control::Drive drive;
drive.definePid(0.4369, 0.6735, 0, 0, 0, 7.4, -10, 10);
drive.defineTransferFunction(polinomialRegression);
drive.defineMotor(13, 1.57, 1.57, 2.0);
drive.defineVelocimeter(17, 0.105);
drive.run(2.5);
```

Realiza mediciones de velocidad a traves de sensores de pulso (Efecto Hall, Infrarojo, etc.)
```c
control::Velocimeter velocimeter;
velocimeter.definePin(17);
velocimeter.defineWheelDiameter(0.105);

while (running)
{
    velocimeter.start();
    velocimeter.waitForUpdate();

    std::cout << velocimeter.getSpeed() << std::endl;
}
```

## 🎮 Uso simple
Clona este repositorio en tu proyecto personal
```bash
git clone https://github.com/TeamAguarai/Control.git
```

Incluye la cabezera única en tu codigo
```c
#include "Control/control.h"
```
> Todo el código fuente está bajo el namespace control

Compila los archivos .cpp incluyendo los de este repositorio con WiringPi
```bash
g++ -std=c++11 -I/control/include $(TU_ARCHIVO).cpp /control/src/*.cpp -o $(TU_ARCHIVO) -lwiringPi
```




## 👨‍🔬Funcionamiento
> ejemplos con imágenes y código de la estructura y funcionamiento del proyecto ...
## ⭐ API
> descripción detallada de las clases y métodos 
