<div align="center">
<h1>pampas</h1>

Controla un vehículo RC como el <a href="https://www.amazon.com/Hobao-HB-VS-C30BU-Hyper-Buggy-Engine/dp/B00MY3ROS2">HYPER VS 1/8 BUGGY NITRO</a> a través de un **Raspberry Pi 3** con *C++*
</div>

## ⚠️ Instalación
> El proyecto está diseñado únicamente para un vehículo con características idénticas al [HYPER VS 1/8 BUGGY NITRO](https://www.amazon.com/Hobao-HB-VS-C30BU-Hyper-Buggy-Engine/dp/B00MY3ROS2) y únicamente puede ser instalado en un Raspberry Pi.

En cualquier carpeta de tu sistema, clona este repositorio y ejecuta el archivo <a href="https://github.com/TeamAguarai/Control/blob/main/install">install</a> (con permisos de administrador)
```bash
git clone https://github.com/TeamAguarai/pampas
cd pampas
chmod +x install scripts/*
sudo ./install
```

## 🎮 Uso
Para poder poder compilar tus proyectos asegurate de incluir las banderas -lpampas -lwiringPi
```bash
g++ main.cpp -o main -lpampas -lwiringPi
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
