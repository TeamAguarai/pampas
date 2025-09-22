<div align="center">
<h1>pampas</h1>

Controla un vehículo RC como el <a href="https://www.amazon.com/Hobao-HB-VS-C30BU-Hyper-Buggy-Engine/dp/B00MY3ROS2">HYPER VS 1/8 BUGGY NITRO</a> a través de un **Raspberry Pi 3** con *C++*
</div>

## ⚠️ Uso de header only
> El proyecto está diseñado únicamente para un vehículo con características idénticas al [HYPER VS 1/8 BUGGY NITRO](https://www.amazon.com/Hobao-HB-VS-C30BU-Hyper-Buggy-Engine/dp/B00MY3ROS2) y únicamente puede ser usado en un Raspberry Pi.

En cualquier carpeta de del Raspberry Pi, clona este repositorio y ejecuta el archivo <a href="https://github.com/TeamAguarai/Control/blob/main/create_header.py">create_header.py</a>
```bash
git clone https://github.com/TeamAguarai/pampas
cd pampas
chmod +x install scripts/*
python3 create_header.py
```
Este comando creara el archivo header only necesario para usar la libreria.

## 🎮 Uso
Para poder poder compilar tus proyectos asegurate de incluir el archivo header only "pampas.hpp" en tu compilacion, asi como la bandera -lwiringPi
```bash
g++ main.cpp -o main -lwiringPi
```



## 🔎 Ejemplos rápidos

Utiliza un controlador PID para gestionar el movimiento del motor
```c
pampas::Drive drive;
drive.setPid(0.4369, 0.6735, 0, 0, 0, 7.4, -10, 10);
drive.setTransferFunction(polinomialRegression);
drive.setMotor(13, 1.57, 1.57, 2.0);
drive.setVelocimeter(17, 0.105);
drive.run(2.5);
```

Realiza mediciones de velocidad a traves de sensores de pulso (Efecto Hall, Infrarojo, etc.)
```c
pampas::Velocimeter velocimeter;
velocimeter.setPin(17);
velocimeter.setWheelDiameter(0.105);

while (running)
{
    velocimeter.start();
    velocimeter.waitForUpdate();

    std::cout << velocimeter.getSpeed() << std::endl;
}
```

## 👨‍🔬Funcionamiento
> ejemplos con imágenes y código de la estructura y funcionamiento del proyecto ...
## ⭐ API
> descripción detallada de las clases y métodos 
