# Aguarai: Sistema de Control para Vehículo Autónomo

## Descripción
La libreria **Aguarai** es un sistema modular diseñado para gestionar y controlar los componentes de un modelo de vehículo autónomo personalizado hecho con un Raspberry Pi 3 programado en C++. Este proyecto permite el manejo de motores, sensores y la captura de datos, proporcionando una solución escalable para desarrollar funcionalidades avanzadas como navegación autónoma.

## Especificaciones
Raspberry Pi 3 Model B V1.2 con la libreria wiringPi, usando la distribucion del pinout BCM.

# Instalación de Aguarai

Aguarai depende de la biblioteca **WiringPi** para gestionar los pines GPIO. Este proyecto incluye un script de instalación que verifica e instala WiringPi automáticamente, si no está disponible en el sistema.

---

## Pasos de Instalación

### 1. Clonar el Repositorio
Clona el repositorio de Aguarai en tu proyecto personal:
```bash
git clone https://github.com/TeamAguarai/Aguarai
```

---

### 2. Ejecutar el Script de Configuración
Si no tienes WiringPi instalado en tu sistema, ejecuta el script `setup.sh` que se encuentra en el repositorio clonado:
```bash
cd aguarai
chmod +x setup.sh
./setup.sh
```

Este script realiza los siguientes pasos:
- Elimina cualquier instalación previa de WiringPi.
- Clona la versión oficial de WiringPi desde GitHub.
- Compila e instala la biblioteca WiringPi.
- Verifica la instalación con el comando `gpio -v`.

---

### 3. Incluir el Archivo de Cabecera
Incluye el archivo principal de encabezado `aguarai.h` en tu código:
```cpp
#include "aguarai/aguarai.h"
```

---

### 4. Compilar Tu Proyecto
Asegúrate de incluir los archivos fuente del proyecto Aguarai en el comando de compilación:
```bash
g++ -std=c++11 -Iaguarai/include main.cpp aguarai/src/*.cpp -o aguarai_program
```

---

## Ejemplo de Código
```cpp
#include "aguarai/aguarai.h"

int main() {
    gpio::setupGpioPinout();
    gpio::pinMode(17, OUTPUT);
    gpio::digitalWrite(17, HIGH);

    Motor motor;
    motor.definePin(18);
    motor.definePulseWidthRange(1.0, 1.5, 2.0);
    motor.setSpeed(1.7);

    return 0;
}
```

---

Con estos pasos, tendrás la librería Aguarai completamente instalada y lista para usarse en tu proyecto.

## Clases y Métodos (DESACTUALIZADO)

### **1. namespace gpio**
**Propósito**: Gestionar los pines GPIO del sistema, incluyendo configuraciones de entrada/salida y señales PWM.

#### **Funciones Principales**
- `void setupGpioPinout()`
  - Inicializa los pines GPIO del sistema.
  - **Uso**:
    ```cpp
    setupGpioPinout();
    ```

- `void pinMode(int pin, PinMode mode)`
  - Configura un pin GPIO como entrada (`INPUT`) o salida (`OUTPUT`).
  - **Parámetros**:
    - `pin`: Número del pin GPIO.
    - `mode`: `INPUT` o `OUTPUT`.
  - **Uso**:
    ```cpp
    pinMode(17, OUTPUT);
    ```

- `void digitalWrite(int pin, int value)`
  - Establece un pin en estado `HIGH` o `LOW`.
  - **Parámetros**:
    - `pin`: Número del pin GPIO.
    - `value`: `HIGH` o `LOW`.
  - **Uso**:
    ```cpp
    digitalWrite(17, HIGH);
    ```

- `void pwmWrite(int pin, float dutyCycle)`
  - Genera una señal PWM en el pin especificado.
  - **Parámetros**:
    - `pin`: Número del pin GPIO.
    - `dutyCycle`: Ciclo de trabajo (valor entre 0 y 1).
  - **Uso**:
    ```cpp
    pwmWrite(18, 0.5); // PWM al 50%
    ```
---

### **2. Clase `Motor`**
**Propósito**: Controlar los motores mediante señales PWM y definir rangos de velocidad personalizados.

#### **Métodos Principales**
- `void definePin(int pin)`
  - Define el pin GPIO que controla el motor.
  - **Parámetros**:
    - `pin`: Número del pin GPIO.

- `void definePulseWidthRange(float min, float neutral, float max)`
  - Establece los valores de ancho de pulso para controlar el motor.
  - **Parámetros**:
    - `min`: Valor mínimo del ancho de pulso.
    - `neutral`: Valor neutral del ancho de pulso.
    - `max`: Valor máximo del ancho de pulso.

- `void setSpeed(float speed)`
  - Configura la velocidad del motor utilizando un ancho de pulso válido.
  - **Parámetros**:
    - `speed`: Valor del ancho de pulso dentro del rango definido.

---

### **3. Clase `PulseWidth`**
**Propósito**: Validar y gestionar rangos de ancho de pulso.

#### **Métodos Principales**
- `void setRange(float min, float neutral, float max)`
  - Define los valores mínimo, neutral y máximo para el rango de pulsos.
  - **Parámetros**:
    - `min`: Valor mínimo.
    - `neutral`: Valor neutral.
    - `max`: Valor máximo.

- `bool isValid(float value)`
  - Verifica si un valor dado está dentro del rango de pulsos definido.
  - **Parámetros**:
    - `value`: Valor a validar.
  - **Retorna**:
    - `true` si el valor está en el rango; de lo contrario, `false`.

---

### **4. Clase `Velocimeter`**
**Propósito**: Calcular la velocidad del vehículo utilizando sensores de pulso.

#### **Métodos Principales**
- `void definePin(int pin)`
  - Establece el pin GPIO para recibir señales del sensor de pulso.
  - **Parámetros**:
    - `pin`: Número del pin GPIO.

- `void defineWheelDiameter(float diameter)`
  - Define el diámetro de las ruedas del vehículo.
  - **Parámetros**:
    - `diameter`: Diámetro de la rueda en metros.

- `void start()`
  - Inicia el cálculo de velocidad basándose en los pulsos detectados.

- `float getSpeed()`
  - Retorna la velocidad calculada en metros por segundo.
  - **Retorna**:
    - Velocidad actual del vehículo.

---

### **5. Clase `Writter`**
**Propósito**: Registrar datos del vehículo en un archivo CSV para análisis posterior.

#### **Métodos Principales**
- `Writter(std::string filename, std::string header)`
  - Constructor que abre un archivo y escribe la cabecera.
  - **Parámetros**:
    - `filename`: Nombre del archivo.
    - `header`: Cabecera con nombres de las columnas.

- `void write_row(float pulseWidth, float speed)`
  - Escribe una fila de datos en el archivo CSV.
  - **Parámetros**:
    - `pulseWidth`: Valor de ancho de pulso.
    - `speed`: Velocidad en m/s.

- `void close()`
  - Cierra el archivo.

---

## Ejemplos de Uso

### Inicializar GPIO y Controlar un Pin
```cpp
#include "gpio.h"

int main() {
    gpio::setupGpioPinout();
    gpio::pinMode(17, gpio::OUTPUT);
    gpio::digitalWrite(17, gpio::HIGH);
    return 0;
}
```

### Controlar un motor
```cpp
#include "Motor.h"

int main() {
    Motor motor;
    motor.definePin(18);
    motor.definePulseWidthRange(1.0, 1.5, 2.0);
    motor.setSpeed(1.7);  // Configurar velocidad
    return 0;
}
```

### Medir Velocidad
```cpp
#include "Velocimeter.h"

int main() {
    Velocimeter velocimeter;
    velocimeter.definePin(22);
    velocimeter.defineWheelDiameter(0.3);  // Diámetro en metros
    velocimeter.start();
    float speed = velocimeter.getSpeed();
    std::cout << "Velocidad actual: " << speed << " m/s\n";
    return 0;
}
```