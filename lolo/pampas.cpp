#include "pampas.h"
#ifdef USING_VSCODE_AS_EDITOR
    #include "PulseWidth.h"
#endif

namespace pampas {
    
void PulseWidth::set(double min, double steady, double max) 
{ 
    this->min = min; 
    this->max = max;
    this->steady = steady; 
}

bool PulseWidth::isDefined() 
{
    if (this->min == -1 || this->max == -1 || this->steady == -1) return false;
    return true;
}

double PulseWidth::validate(double pulseWidth) 
{
    if (pulseWidth > this->max) return this->max;
    if (pulseWidth < this->min) return this->min;
    return pulseWidth;
}

}
#ifdef USING_VSCODE_AS_EDITOR
    #include "gpio.h"
#endif


namespace pampas {

namespace gpio {

    void setupGpioPinout() {
        ::wiringPiSetupGpio();  
    }

    void pwmWrite(int pin, double pulseWidthMs) {

        // AGUARAI STEADY STATE
        if (pulseWidthMs > 2.0) pulseWidthMs = 2.0;
        else if (pulseWidthMs < 1.0) pulseWidthMs = 1.0;

        
        // Calculate the range (resolution) and the divisor to adjust the frequency
        double periodMs = 1000.0 / PWM_FREQUENCY; // Period in milliseconds
        int range = 1024; // PWM resolution (default in WiringPi)
        int divisor = static_cast<int>(19200000.0 / (PWM_FREQUENCY * range)); // PWM clock at 19.2 MHz

        // Set the divisor and range
        ::pwmSetMode(0);      // "Mark-Space" mode for precision
        ::pwmSetRange(range);           // Set the range for the duty cycle
        ::pwmSetClock(divisor);         // Set the divisor to adjust the frequency

        // Calculate the duty cycle corresponding to the desired pulse width
        int dutyCycle = static_cast<int>((pulseWidthMs / periodMs) * range);

        // Set the duty cycle
        ::pwmWrite(pin, dutyCycle);
    }


    void pinMode(int pin, int mode) {
        ::pinMode(pin, mode);
    }

    void digitalWrite(int pin, int value) {
        ::digitalWrite(pin, value);
    }

    int digitalRead(int pin) {
        return ::digitalRead(pin);
    }

    void onInterrupt (int pin, int edgeType,  void (*function)(void)) {
        ::wiringPiISR(pin, edgeType, function);
    }

    void stopOnInterrupt(int pin) {
        ::wiringPiISRStop(pin); 
    }

    void reset() // pines pwm a estado estacionario
    {
        // pines pwm a estado estacionario
        ::pinMode(13, 2);
        ::pwmWrite(13, 1.5);

        ::pinMode(18, 2);
        ::pwmWrite(18, 1.5);
        
    }

}

}
#ifdef USING_VSCODE_AS_EDITOR
    #include "Motor.h"
#endif


namespace pampas {
    
Motor* motorInstance = nullptr;

Motor::Motor() {
    motorInstance = this;
    std::signal(SIGINT, [](int) {
        motorInstance->cleanup();  
    });
}

Motor::~Motor() 
{
    this->cleanup();
}

void Motor::cleanup() 
{
    this->setPulseWidth(this->pulseWidth.steady);
}

void Motor::setPin(int pin) 
{
    this->pin = pin;
    gpio::pinMode(pin, PWM_OUTPUT); // PWM_OUTPUT is a wiringPi constant
}

void Motor::setPulseWidthRange(double min, double steady, double max) {
    this->pulseWidth.set(min, steady, max);
}

void Motor::setPulseWidth(double pulseWidth) 
{
    if (this->pulseWidth.isDefined() == false) throw std::invalid_argument( "Faltan definir los valores de ancho de pulso." );
    if (this->pin == -1) throw std::invalid_argument( "Faltan definir el pin del motor" );
    gpio::pwmWrite(this->pin, this->pulseWidth.validate(pulseWidth));
}


/* Analizar posible implementacion
void Motor::runForMilliseconds(int milliseconds, double pulseWidthMs) {
    std::thread([=]() {
        this->setPulseWidth(pulseWidthMs); 
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }).detach();
}
*/


}
#ifdef USING_VSCODE_AS_EDITOR
    #include "operations.h"
#endif

namespace pampas {

template <typename T>
T remap(T value, T in_min, T in_max, T out_min, T out_max) {
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

void delay(int ms) {
	::delay(ms); // wiringPi function
}

// Redondeo para float
float round(float num, int decimals) {
    float factor = std::pow(10.0f, decimals);
    return std::round(std::round(num * (factor * 10.0f)) / 10.0f) / factor;
}

// Redondeo para double
double round(double num, int decimals) {
    double factor = std::pow(10.0, decimals);
    return std::round(std::round(num * (factor * 10.0)) / 10.0) / factor;
}

// Redondeo para enteros (No afecta valores enteros)
int round(int num, int) {
    return num;
}

}


#ifdef USING_VSCODE_AS_EDITOR
    #include "Steer.h"
#endif

namespace pampas {

void Steer::setPulseWidthRange(double min, double steady, double max)
{
    this->servo.setPulseWidthRange(min, steady, max);
}

void Steer::setPin(int pin)
{
    this->servo.setPin(pin);
}

void Steer::steer(double value, double proportionalConstant) // -1 < value < 1
{
    if (value > this->max || value < this->min) throw std::invalid_argument( "El valor esta fuera del rango: -1 < value < 1" );

    double pulseWidthValue = pampas::remap(value, this->min, this->max, this->servo.pulseWidth.min, this->servo.pulseWidth.max);
    
    this->servo.setPulseWidth(pulseWidthValue);
}

}
#ifdef USING_VSCODE_AS_EDITOR
    #include "Exception.h"
#endif


namespace pampas {

std::string Exception::formatMessage(const std::string& message, const std::string& file, int line) {
    std::ostringstream oss;
    oss << "\nError: " << message << "\n"
        << "Archivo: " << file << "\n"
        << "Linea: " << line;
    return oss.str();
}

void raiseError(std::string msg) {
    throw Exception(msg, __FILE__, __LINE__);
}

}

#ifdef USING_VSCODE_AS_EDITOR
    #include "LowPass.h"
#endif


namespace pampas {

template<typename T>
LowPass<T>::LowPass() : prevOutput(static_cast<T>(0)) {} // define el tipo de dato que se usara

template<typename T>
void LowPass<T>::setAlpha(T value) {
    if (value < static_cast<T>(0) || value > static_cast<T>(1)) {
        throw std::invalid_argument("Coeficiente de suavizado debe estar en el rango [0,1]");
    }
    this->alphaDefined = true;
    alpha = value;
}

template<typename T>
void LowPass<T>::setInitialValue(T value) {
    this->prevOutput = value;
}

template<typename T>
T LowPass<T>::filter(T input) {
    if (!this->alphaDefined) {
        throw std::runtime_error("Coeficiente de suavizado no ha sido inicializado o tiene un valor inválido.");
    }

    T output = alpha * input + (static_cast<T>(1) - alpha) * prevOutput;
    prevOutput = output;
    return output;
}

// Instanciaciones explícitas (necesario si se usa fuera del .h)

template class LowPass<double>;
template class LowPass<float>;
template class LowPass<int>;

} // namespace pampas

#ifdef USING_VSCODE_AS_EDITOR
    #include "Velocimeter.h"
#endif

namespace pampas {

Velocimeter* velocimeterInstance = nullptr;

Velocimeter::Velocimeter() 
{
    pampas::gpio::setupGpioPinout();
    velocimeterInstance = this;

    // prepara el cleanup en caso de un programa abortado !! REVISAR
    std::signal(SIGINT, [](int) {
        velocimeterInstance->cleanup();
    });    

}

Velocimeter::~Velocimeter() 
{
    this->cleanup();
}

void Velocimeter::cleanup() {
    gpio::stopOnInterrupt(velocimeterInstance->pin);
}

void Velocimeter::setPin(int pin) 
{
    this->pin = pin;
}

void Velocimeter::setWheelDiameter(double wheelDiameter) 
{
    this->wheelDiameter = wheelDiameter;
    this->wheelCircumference = 2 * (wheelDiameter/2) * 3.1416;
}

void Velocimeter::setAlpha(double value)
{
    this->filter.setAlpha(value);
}

void Velocimeter::pulseHandlerWrapper() 
{
    velocimeterInstance->pulseHandler();
}

void Velocimeter::pulseHandler() 
{
    if (this->started == false) return;
    
    // this->distance += this->wheelCircumference;

    clock_gettime(CLOCK_MONOTONIC, &this->endTime);
    this->timeInterval = (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_nsec - startTime.tv_nsec) / 1e9;
    this->speed = (timeInterval > 0) ? (this->wheelCircumference / this->timeInterval) : 0.0;
    clock_gettime(CLOCK_MONOTONIC, &this->startTime);
    
    this->udpated = true;    
}
 
void Velocimeter::start() 
{
    this->udpated = false;
    if (this->started == true) return;

    clock_gettime(CLOCK_MONOTONIC, &this->startTime);

    gpio::pinMode(this->pin, INPUT);
    gpio::onInterrupt(this->pin, INT_EDGE_RISING, &pulseHandlerWrapper);
    
    this->started = true;
}

double Velocimeter::getUpdateTimeInterval() 
{
    return this->timeInterval;
}

double Velocimeter::getSpeed() 
{
    // return this->speed;
    return this->filter.filter(this->speed); // valor filtrado
}

double Velocimeter::getDistance() 
{
    return this->distance;
}

void Velocimeter::resetDistance() 
{
    this->distance = 0;
}

void Velocimeter::waitForUpdate(double timeoutSeconds) 
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end;
    double timeDifference = 0;

    while (!this->udpated && timeDifference <= timeoutSeconds) {
        end = std::chrono::steady_clock::now();
        timeDifference = std::chrono::duration_cast<std::chrono::seconds>(end - begin).count();
        delay(10); // para evitar sobrecarga
    }

    if (timeDifference >= timeoutSeconds) {
        this->speed = 0; // cuando se cumple el timeout, la velocidad se supone que es nula
        // Como el sensor nunca detecto pulso, nunca se registra un intervalo de tiempo
        // para eso se le da un nuevo valor que es el timeOut 
        this->timeInterval = timeoutSeconds;
    }
    
}

}
#ifdef USING_VSCODE_AS_EDITOR
    #include "PID.h"
#endif

/* Creditos: https://github.com/pms67/PID/blob/master/PID.c */
namespace pampas {

PID::PID() 
{
    integrator_ = 0.0f;
    prev_error_ = 0.0f;
    differentiator_ = 0.0f;
    prev_measurement_ = 0.0f;
    out_ = 0.0f;
}

void PID::reset() {
    integrator_ = 0.0f;
    prev_error_ = 0.0f;
    differentiator_ = 0.0f;
    prev_measurement_ = 0.0f;
    out_ = 0.0f;
}

double PID::calculate(double setpoint, double measurement, double sample_time_) 
{
    if (!gains_defined_) throw std::invalid_argument( "Faltan definir las ganancias del controlador PID" );
    if (!params_defined_) throw std::invalid_argument( "Faltan definir los parametros del controlador PID" );

    double error = setpoint - measurement;
    double proportional = kp_ * error;
    integrator_ += 0.5f * ki_ * sample_time_ * (error + prev_error_);

    if (integrator_ > max_output_int_) {
        integrator_ = max_output_int_;
    } else if (integrator_ < min_output_int_) {
        integrator_ = min_output_int_;
    }

    // Derivativo (filtro de paso bajo)
    differentiator_ = -(2.0f * kd_ * (measurement - prev_measurement_) +
                       (2.0f * tau_ - sample_time_) * differentiator_) /
                     (2.0f * tau_ + sample_time_);

    out_ = proportional + integrator_ + differentiator_;

    if (out_ > max_output_) {
        out_ = max_output_;
    } else if (out_ < min_output_) {
        out_ = min_output_;
    }

    prev_error_ = error;
    prev_measurement_ = measurement;

    return out_;
}

void PID::setGains(double kp, double ki, double kd) 
{
    this->gains_defined_ = true;
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;
}

void PID::setParameters(double tau_, double min_output_, double max_output_, double min_output_int_, double max_output_int_) 
{
    this->params_defined_ = true;
    this->tau_ = tau_;
    this->min_output_ = min_output_;
    this->max_output_ = max_output_;
    this->min_output_int_ = min_output_int_;
    this->max_output_int_ = max_output_int_;
}

}
#ifdef USING_VSCODE_AS_EDITOR
    #include "Conversion.h"
#endif

namespace pampas {

bool Conversion::isDefined() 
{
    return this->defined;
}

void Conversion::set(std::function<double(double)> func) 
{
    this->defined = true;
    this->transferFunc = func;
    
}

double Conversion::convert(double input) 
{
    if (!this->defined) throw std::runtime_error("Función de conversión no definida.");
    return this->transferFunc(input);
}

}


#ifdef USING_VSCODE_AS_EDITOR
    #include "Drive.h"
#endif
#include <iostream>


namespace pampas {

Drive* driveInstance = nullptr;


Drive::Drive()
{
    pampas::gpio::setupGpioPinout();
}

void Drive::setPid(double kp, double ki, double kd, double tau, double minOutput, double maxOutput, double minOutputInt, double maxOutputInt) 
{
    this->pid.setGains(kp, ki, kd);
    this->pid.setParameters(tau, minOutput, maxOutput, minOutputInt, maxOutputInt);
}

void Drive::setVelocimeter(int pin, double wheelDiameterCM, double alpha) 
{
    this->velocimeter.setPin(pin);
    this->velocimeter.setWheelDiameter(wheelDiameterCM);
    this->velocimeter.setAlpha(alpha);
}


void Drive::setMotor(int pin, double pulseWidthMin, double pulseWidthSteady, double pulseWidthMax) 
{
    this->motor.setPin(pin);
    this->motor.setPulseWidthRange(pulseWidthMin, pulseWidthSteady, pulseWidthMax);
}

void Drive::setTransferFunction(std::function<double(double)> func) 
{
    this->MsToPulseWidth.set(func);
}

void Drive::stop() {
    this->running = false;
    if (this->control.joinable()) this->control.join();
}


double Drive::update_model_reference(double v_ref, double dt, double w_n, double z) {

    // Constantes internas (pueden ser ajustadas según tu sistema)
    double wn = w_n;
    double zeta = z;

    // Ecuación del modelo de referencia (sistema de segundo orden)
    double ddv_m = (wn * wn) * v_ref - 2.0 * zeta * wn * dv_m_ - (wn * wn) * v_m_;
    // std::cout << (wn * wn) * v_ref << " - ";
    // std::cout << 2.0 * zeta * wn * dv_m << " - ";
    // std::cout << (wn * wn) * v_m<< "\n\n";

    dv_m_ += ddv_m * dt;
    v_m_ += dv_m_ * dt;

    return v_m_;
}

void Drive::controlledSpeed(double speed, double w_n, double z) 
{
    
    double kp_adap = 0.0;
    double ki_adap = 0.0;
    double kd_adap = 0.0;

    double err_int = 0.0;

    double prev_error = 0.0;
    
    double g_p = 0.0;
    double g_d = 0.0;
    double g_i = 0.0;
    
    std::cout << "Ingrese el valor de gamma_p: ";
    std::cin >> g_p;
    
    std::cout << "Ingrese el valor de gamma_i: ";
    std::cin >> g_i;

    std::cout << "Ingrese el valor de gamma_d: ";
    std::cin >> g_d;

    std::cout << "Ingrese el valor de tau: ";
    double tau_filter;                      // constante de tiempo del filtro
    std::cin >> tau_filter;
    
    std::cout << "PID ADAPTATIVO: setpoint(" << speed << ")  wn(" << w_n << ") zeta(" << z << ") g_p(" << g_p << ") g_i(" << g_i << ")  g_d(" << g_d << ")\n";
    v_m_ = 0.0;
    dv_m_ = 0.0;
    
    this->pid.reset();
    
    this->motor.setPulseWidth(this->MsToPulseWidth.convert(speed));
    
    clock_gettime(CLOCK_MONOTONIC, &this->velocimeter.startTime);
    
    
    auto inicio = std::chrono::steady_clock::now();
    


    double deriv_error_filtered = 0; 

    while (true) {
        this->velocimeter.start();
        this->velocimeter.waitForUpdate();
        
        double measurement = this->velocimeter.getSpeed();
        double dt = this->velocimeter.getUpdateTimeInterval();

        double model_reference_setpoint = update_model_reference(speed, dt, w_n, z);
        double error = model_reference_setpoint - measurement;

        
        kp_adap += ((-g_p) * (0.0313*error + 0.0553*(error - prev_error)/dt) * error) * dt;
        if (kp_adap > 100) kp_adap = 100.0;
        if (kp_adap < 0) kp_adap = 0;


        // Derivada cruda
        double raw_deriv = (error - prev_error) / dt;

        // Filtro exponencial de primer orden
        deriv_error_filtered = (tau_filter * deriv_error_filtered + dt * raw_deriv) / (tau_filter + dt);

        kd_adap += ((-g_d) * (0.0313*error + 0.0553*(deriv_error_filtered)/dt) * (deriv_error_filtered)/dt) * dt;
        if (kd_adap > 50) kd_adap = 50.0;
        // if (kd_adap < 0) kd_adap = 0;
        
        err_int += error * dt;
        ki_adap += (-g_i) * (0.0313*error + 0.0553*(error - prev_error)/dt) * err_int;
        
        this->pid.setGains(kp_adap, ki_adap, kd_adap);

        prev_error = error;
        
        double newSpeed = this->pid.calculate(
            model_reference_setpoint, 
            measurement, 
            dt
        );

        this->motor.setPulseWidth(this->MsToPulseWidth.convert(newSpeed));

        std::cout << "SetPoint: " << speed << " | MD SetPoint: " << model_reference_setpoint << " | Velocimetro: " << measurement << " | Velocidad Nueva: " << newSpeed << " | KP(" << this->pid.kp_ <<")" << " KI(" << this->pid.ki_ << ")" << " KD(" << this->pid.kd_ << ") dt(" << dt << ")\n";
    }

}

void Drive::run(double speed, double w_n, double z) 
{
    // if (speed == this->runningSpeed) return; // evita que se cree un nuevo thread para una velocidad ya definida

    // this->stop();
    this->runningSpeed = speed;
    this->running = true;
    
    this->controlledSpeed(speed, w_n, z);
    // this->control = std::thread(&Drive::controlledSpeed, this, speed, w_n, z);
}

}
#ifdef USING_VSCODE_AS_EDITOR
    #include "Writer.h"
#endif

namespace pampas {
    
Writer::Writer(std::string filename, std::string header, std::string delim) : is_open(false), delimiter(delim)
{
    file.open(filename);
    if (file.is_open()) {
        is_open = true;
        file << header << "\n";
    } else {
        std::cerr << "Error: No se pudo abrir el archivo " << filename << "\n";
    }
}

Writer::~Writer() 
{
    if (is_open) file.close();
}

void Writer::write_row(const std::vector<std::string>& data) 
{
    if (!is_open) {
        std::cerr << "Error: El archivo no está abierto.\n";
        return;
    } 
    std::ostringstream oss;
    for (size_t i = 0; i < data.size(); ++i) {
        oss << data[i];
        if (i != data.size() - 1) {
            oss << delimiter;
        }
    }
    file << oss.str() << "\n";
}

void Writer::close() 
{
    if (is_open) {
        file.close();
        is_open = false;
    }
}

}
/*
* _dev: funcionalidad para desarrollo interno
*/


#ifdef USING_VSCODE_AS_EDITOR
    #include "_dev.h"
#endif

namespace pampas {

void hello() 
{
    std::cout << "HELLO FROM PAMPAS!" << std::endl;
}

}
// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>
//
// Updated:
// 14/04/2014 by Gregory Dymare <gregd72002@gmail.com> - removed C++ dependencies
//
// Changelog:
//     2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//                - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
//     2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
//     2011-10-03 - added automatic Arduino version detection for ease of use
//     2011-10-02 - added Gene Knight's NBWire TwoWire class implementation with small modifications
//     2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
//     2011-08-03 - added optional timeout parameter to read* methods to easily change from default
//     2011-08-02 - added support for 16-bit registers
//                - fixed incorrect Doxygen comments on some methods
//                - added timeout value for read operations (thanks mem @ Arduino forums)
//     2011-07-30 - changed read/write function structures to return success or byte counts
//                - made all methods static for multi-device memory savings
//     2011-07-28 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifdef USING_VSCODE_AS_EDITOR
    #include "I2Cdev.h"
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

namespace pampas {

    
    /** Default timeout value for read operations.
 * Set this to 0 to disable timeout detection.
 */
uint16_t readTimeout = 0;
/** Default constructor.
 */

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data) {
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = readWord(devAddr, regAddr, &w)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
    return readBytes(devAddr, regAddr, 1, data);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data) {
    return readWords(devAddr, regAddr, 1, data);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
    int8_t count = 0;
#ifdef DEBUG
printf("read %#x %#x %u\n",devAddr,regAddr,length);
#endif
int fd = open("/dev/i2c-1", O_RDWR);

if (fd < 0) {
    fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
    return(-1);
}
if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
    fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
    close(fd);
    return(-1);
}
if (write(fd, &regAddr, 1) != 1) {
    fprintf(stderr, "Failed to write reg: %s\n", strerror(errno));
    close(fd);
    return(-1);
}
count = read(fd, data, length);
if (count < 0) {
    fprintf(stderr, "Failed to read device(%d): %s\n", count, strerror(errno));
    close(fd);
    return(-1);
} else if (count != length) {
    fprintf(stderr, "Short read  from device, expected %d, got %d\n", length, count);
    close(fd);
    return(-1);
}
close(fd);

return count;
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of words read (0 indicates failure)
 */
int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data) {
    int8_t count = 0;
    
    printf("ReadWords() not implemented\n");
    // Use readBytes() and potential byteswap
    *data = 0; // keep the compiler quiet
    
    return count;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
int writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
int writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t w;
    readWord(devAddr, regAddr, &w);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
int writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
    } else {
        return -1;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
int writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask byte
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(devAddr, regAddr, &w) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(devAddr, regAddr, w);
    } else {
        return -1;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
int writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBytes(devAddr, regAddr, 1, &data);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
int writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    return writeWords(devAddr, regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
int writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
    int8_t count = 0;
    uint8_t buf[128];
    int fd;
    
    #ifdef DEBUG
    printf("write %#x %#x\n",devAddr,regAddr);
    #endif
    if (length > 127) {
        fprintf(stderr, "Byte write count (%d) > 127\n", length);
        return -1;
    }
    
    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
        fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    buf[0] = regAddr;
    memcpy(buf+1,data,length);
    count = write(fd, buf, length+1);
    if (count < 0) {
        fprintf(stderr, "Failed to write device(%d): %s\n", count, strerror(errno));
        close(fd);
        return -1;
    } else if (count != length+1) {
        fprintf(stderr, "Short write to device, expected %d, got %d\n", length+1, count);
        close(fd);
        return -1;
    }
    close(fd);
    
    return 0;
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
int writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {
    int8_t count = 0;
    uint8_t buf[128];
    int i, fd;
    
    // Should do potential byteswap and call writeBytes() really, but that
    // messes with the callers buffer
    
    if (length > 63) {
        fprintf(stderr, "Word write count (%d) > 63\n", length);
        return -1;
    }
    
    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
        fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    buf[0] = regAddr;
    for (i = 0; i < length; i++) {
        buf[i*2+1] = data[i] >> 8;
        buf[i*2+2] = data[i];
    }
    count = write(fd, buf, length*2+1);
    if (count < 0) {
        fprintf(stderr, "Failed to write device(%d): %s\n", count, strerror(errno));
        close(fd);
        return -1;
    } else if (count != length*2+1) {
        fprintf(stderr, "Short write to device, expected %d, got %d\n", length+1, count);
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

} // namespace pampas
#ifdef USING_VSCODE_AS_EDITOR
#include "MPU9250.h"
#endif

namespace pampas
{

/* deletes spaces from a string */
std::string trim(const std::string& str) {
    auto start = str.begin();
    while (start != str.end() && std::isspace(*start)) {
        start++;
    }

    auto end = str.end();
    do {
        end--;
    } while (std::distance(start, end) > 0 && std::isspace(*end));

    return std::string(start, end + 1);
}

/* mpu9250 constructor */
MPU9250::MPU9250(){
    _address = 0x68; // I2C address
    _useSPI = false; // set to use I2C
    _axb = 0.0f;
    _ayb = 0.0f;
    _azb = 0.0f;

    _gxb = 0.0f;
    _gyb = 0.0f;
    _gzb = 0.0f;

    _hxb = 0.0f;
    _hyb = 0.0f;
    _hzb = 0.0f;

    _gyroOutputFilters[0].setAlpha(_gyroFiltersAlpha);
    _gyroOutputFilters[1].setAlpha(_gyroFiltersAlpha);
    _gyroOutputFilters[2].setAlpha(_gyroFiltersAlpha);
}

/* starts communication with the MPU-9250 */
int MPU9250::begin(){
    // select clock source to gyro
    if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
        return -1;
    }
    // enable I2C master mode
    if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
        return -2;
    }
    // set the I2C bus speed to 400 kHz
    if(writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
        return -3;
    }
    // set AK8963 to Power Down
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
    // reset the MPU9250
    writeRegister(PWR_MGMNT_1,PWR_RESET);
    // wait for MPU-9250 to come back up
    delay(1);
    // reset the AK8963
    writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
    // select clock source to gyro
    if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
        return -4;
    }
    // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
    if((whoAmI() != 113)&&(whoAmI() != 115)){
        return -5;
    }
    // enable accelerometer and gyro
    if(writeRegister(PWR_MGMNT_2,SEN_ENABLE) < 0){
        return -6;
    }
    // setting accel range to 16G as default
    if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
        return -7;
    }
    _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
    _accelRange = ACCEL_RANGE_16G;
    // setting the gyro range to 2000DPS as default
    if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
        return -8;
    }
    _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
    _gyroRange = GYRO_RANGE_2000DPS;
    // setting bandwidth to 184Hz as default
    if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){
        return -9;
    }
    if(writeRegister(CONFIG,GYRO_DLPF_184) < 0){ // setting gyro bandwidth to 184Hz
        return -10;
    }
    _bandwidth = DLPF_BANDWIDTH_184HZ;
    // setting the sample rate divider to 0 as default
    if(writeRegister(SMPDIV,0x00) < 0){
        return -11;
    }
    _srd = 0;
    // enable I2C master mode
    if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
        return -12;
    }
    // set the I2C bus speed to 400 kHz
    if( writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
        return -13;
    }
    // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
    if( whoAmIAK8963() != 72 ){
        return -14;
    }
    /* get the magnetometer calibration */
    // set AK8963 to Power Down
    if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
        return -15;
    }
    delay(100); // long wait between AK8963 mode changes
    // set AK8963 to FUSE ROM access
    if(writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM) < 0){
        return -16;
    }
    delay(100); // long wait between AK8963 mode changes
    // read the AK8963 ASA registers and compute magnetometer scale factors
    readAK8963Registers(AK8963_ASA,3,_buffer);
    _magScaleX = ((((float)_buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    _magScaleY = ((((float)_buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    _magScaleZ = ((((float)_buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
    // std::cout<<__FILE__<<__LINE__<<"  "<<_magScaleX<<"\t"<<_magScaleY<<"\t"<<_magScaleZ<<"\n";
    // set AK8963 to Power Down
    if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
        return -17;
    }
    delay(100); // long wait between AK8963 mode changes
    // set AK8963 to 16 bit resolution, 100 Hz update rate
    if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
        return -18;
    }
    delay(100); // long wait between AK8963 mode changes
    // select clock source to gyro
    if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
        return -19;
    }
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL,7,_buffer);


    // estimate gyro bias
    // if (calibrateGyro() < 0) {
    //     std::cerr<<__FILE__<<__LINE__<<": error calibrateGyro."<<std::endl;
    //     return -21;
    // }


    // if (calibrateAccel() < 0) {
    //     std::cerr<<__FILE__<<__LINE__<<": error calibrateAccel."<<std::endl;
    //     return -20;
    // }
/*
    if (calibrateMag() < 0) {
        std::cerr<<__FILE__<<__LINE__<<": error calibrateMag."<<std::endl;
        return -22;
    }
 */
    // successful init, return 1
    return 1;
}

/* writes a byte to MPU9250 register given a register address and data */
int MPU9250::writeRegister(uint8_t subAddress, uint8_t data){

    writeByte(_address, subAddress, data);

    delay(10); 

    /* read back the register */
    readRegisters(subAddress, 1, _buffer);
    /* check the read back register against the written register */

    if(_buffer[0] == data) {
        return 1;
    }
    else{
        return -1;
    }
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
int MPU9250::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
    if ( count == readBytes(_address, subAddress, count, dest)){
        return 0;
    }
    else {
        return -1;
    }

}

/* calibrate gyro */
void MPU9250::calibrate() {
    std::cout << "Inicio proceso de calibracion (10 segundos). MANTENGA AL SENSOR PLANO Y QUIETO\n";
    this->calibrateGyro(10);
    std::cout << "Fin Calibracion.\n";
}

/* calibrate gyro and save calibration data for later use */
void MPU9250::calibrate(std::string calibration_output_filename) {
    std::cout << "Inicio proceso de calibracion (10 segundos). MANTENGA AL SENSOR PLANO Y QUIETO\n";
    
    this->calibrateGyro(10);
    this->saveCalibration(calibration_output_filename);

    std::cout << "Fin Calibracion.\n";
}
  
/* saves the calibration data in a .txt file */
int MPU9250::saveCalibration(std::string path_to_save_calibration_output) {
    Writer writer(path_to_save_calibration_output, "Sensor,Bias,Scale");
    writer.write_row({"Samples for Gyro: ", std::to_string(_gyroNumSamples)});
    writer.write_row({"Gyro_Bias_X", std::to_string(getGyroBiasX_rads()), "1.0"});
    writer.write_row({"Gyro_Bias_Y", std::to_string(getGyroBiasY_rads()), "1.0"});
    writer.write_row({"Gyro_Bias_Z", std::to_string(getGyroBiasZ_rads()), "1.0"});

    // writer.write_row({"Accel_X", std::to_string(_axb), std::to_string(_axs)});
    // writer.write_row({"Accel_Y", std::to_string(_ayb), std::to_string(_ays)});
    // writer.write_row({"Accel_Z", std::to_string(_azb), std::to_string(_azs)});

    // writer.write_row({"Mag_X", std::to_string(_hxb), std::to_string(_hxs)});
    // writer.write_row({"Mag_Y", std::to_string(_hyb), std::to_string(_hys)});
    // writer.write_row({"Mag_Z", std::to_string(_hzb), std::to_string(_hzs)});

    writer.close();
    return 1;
}

/* calibrate mpu from a data file */
int MPU9250::loadCalibration(std::string file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error: No se pudo abrir el archivo para cargar la calibración." << std::endl;
        return -1;
    }

    std::string line;
    // Saltar la primera línea (encabezado)
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string sensor, bias_str, scale_str;

        // Leer columnas
        if (std::getline(ss, sensor, ',') && std::getline(ss, bias_str, ',') && std::getline(ss, scale_str, ',')) {
            try {
                // Limpiar cadenas
                sensor = trim(sensor);
                bias_str = trim(bias_str);
                scale_str = trim(scale_str);

                // Convertir a float
                size_t idx1 = 0, idx2 = 0;
                float bias = std::stof(bias_str, &idx1);
                float scale = std::stof(scale_str, &idx2);

                if (idx1 != bias_str.size() || idx2 != scale_str.size()) {
                    std::cerr << "Error: Valor no numérico en línea: " << line << std::endl;
                    continue;
                }

                // Asignar valores
                if (sensor == "Gyro_X") {
                    setGyroBiasX_rads(bias);
                } else if (sensor == "Gyro_Y") {
                    setGyroBiasY_rads(bias);
                } else if (sensor == "Gyro_Z") {
                    setGyroBiasZ_rads(bias);
                }
                // } else if (sensor == "Accel_X") {
                //     _axb = bias;
                //     _axs = scale;
                // } else if (sensor == "Accel_Y") {
                //     _ayb = bias;
                //     _ays = scale;
                // } else if (sensor == "Accel_Z") {
                //     _azb = bias;
                //     _azs = scale;
                // } else if (sensor == "Mag_X") {
                //     _hxb = bias;
                //     _hxs = scale;
                // } else if (sensor == "Mag_Y") {
                //     _hyb = bias;
                //     _hys = scale;
                // } else if (sensor == "Mag_Z") {
                //     _hzb = bias;
                //     _hzs = scale;
                // }
            } catch (const std::invalid_argument& e) {
                std::cerr << "Error: Valor no válido en línea: " << line << std::endl;
                continue;
            } catch (const std::out_of_range& e) {
                std::cerr << "Error: Valor fuera de rango en línea: " << line << std::endl;
                continue;
            }
        } else {
            std::cerr << "Error: Formato de línea no válido: " << line << std::endl;
        }
    }

    file.close();
    return 1;
}

/* sets the accelerometer full scale range to values other than default */
int MPU9250::setAccelRange(AccelRange range) {
    // use low speed SPI for register setting
    _useSPIHS = false;
    switch(range) {
    case ACCEL_RANGE_2G: {
        // setting the accel range to 2G
        if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G) < 0){
            return -1;
        }
        _accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
        break;
    }
    case ACCEL_RANGE_4G: {
        // setting the accel range to 4G
        if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G) < 0){
            return -1;
        }
        _accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
        break;
    }
    case ACCEL_RANGE_8G: {
        // setting the accel range to 8G
        if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_8G) < 0){
            return -1;
        }
        _accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
        break;
    }
    case ACCEL_RANGE_16G: {
        // setting the accel range to 16G
        if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
            return -1;
        }
        _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
        break;
    }
    }
    _accelRange = range;
    return 1;
}

/* sets the gyro full scale range to values other than default */
int MPU9250::setGyroRange(GyroRange range) {
    // use low speed SPI for register setting
    _useSPIHS = false;
    switch(range) {
    case GYRO_RANGE_250DPS: {
        // setting the gyro range to 250DPS
        if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS) < 0){
            return -1;
        }
        _gyroScale = 250.0f/32767.5f * _d2r; // setting the gyro scale to 250DPS
        break;
    }
    case GYRO_RANGE_500DPS: {
        // setting the gyro range to 500DPS
        if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS) < 0){
            return -1;
        }
        _gyroScale = 500.0f/32767.5f * _d2r; // setting the gyro scale to 500DPS
        break;
    }
    case GYRO_RANGE_1000DPS: {
        // setting the gyro range to 1000DPS
        if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS) < 0){
            return -1;
        }
        _gyroScale = 1000.0f/32767.5f * _d2r; // setting the gyro scale to 1000DPS
        break;
    }
    case GYRO_RANGE_2000DPS: {
        // setting the gyro range to 2000DPS
        if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
            return -1;
        }
        _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
        break;
    }
    }
    _gyroRange = range;
    return 1;
}

/* sets the DLPF bandwidth to values other than default */
int MPU9250::setDlpfBandwidth(DlpfBandwidth bandwidth) {
    // use low speed SPI for register setting
    _useSPIHS = false;
    switch(bandwidth) {
    case DLPF_BANDWIDTH_184HZ: {
        if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ // setting accel bandwidth to 184Hz
            return -1;
        }
        if(writeRegister(CONFIG,GYRO_DLPF_184) < 0){ // setting gyro bandwidth to 184Hz
            return -2;
        }
        break;
    }
    case DLPF_BANDWIDTH_92HZ: {
        if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_92) < 0){ // setting accel bandwidth to 92Hz
            return -1;
        }
        if(writeRegister(CONFIG,GYRO_DLPF_92) < 0){ // setting gyro bandwidth to 92Hz
            return -2;
        }
        break;
    }
    case DLPF_BANDWIDTH_41HZ: {
        if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_41) < 0){ // setting accel bandwidth to 41Hz
            return -1;
        }
        if(writeRegister(CONFIG,GYRO_DLPF_41) < 0){ // setting gyro bandwidth to 41Hz
            return -2;
        }
        break;
    }
    case DLPF_BANDWIDTH_20HZ: {
        if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_20) < 0){ // setting accel bandwidth to 20Hz
            return -1;
        }
        if(writeRegister(CONFIG,GYRO_DLPF_20) < 0){ // setting gyro bandwidth to 20Hz
            return -2;
        }
        break;
    }
    case DLPF_BANDWIDTH_10HZ: {
        if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_10) < 0){ // setting accel bandwidth to 10Hz
            return -1;
        }
        if(writeRegister(CONFIG,GYRO_DLPF_10) < 0){ // setting gyro bandwidth to 10Hz
            return -2;
        }
        break;
    }
    case DLPF_BANDWIDTH_5HZ: {
        if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_5) < 0){ // setting accel bandwidth to 5Hz
            return -1;
        }
        if(writeRegister(CONFIG,GYRO_DLPF_5) < 0){ // setting gyro bandwidth to 5Hz
            return -2;
        }
        break;
    }
    }
    _bandwidth = bandwidth;
    return 1;
}

/* sets the sample rate divider to values other than default */
int MPU9250::setSrd(uint8_t srd) {
    // use low speed SPI for register setting
    _useSPIHS = false;
    /* setting the sample rate divider to 19 to facilitate setting up magnetometer */
    if(writeRegister(SMPDIV,19) < 0){ // setting the sample rate divider
        return -1;
    }
    if(srd > 9){
        // set AK8963 to Power Down
        if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
            std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
            return -2;
        }
        delay(100); // long wait between AK8963 mode changes
        // set AK8963 to 16 bit resolution, 8 Hz update rate
        if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1) < 0){
            std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
            return -3;
        }
        delay(100); // long wait between AK8963 mode changes
        // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
        readAK8963Registers(AK8963_HXL,7,_buffer);
    } else {
        // set AK8963 to Power Down
        if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
            std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
            return -2;
        }
        delay(100); // long wait between AK8963 mode changes
        // set AK8963 to 16 bit resolution, 100 Hz update rate
        if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
            std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
            return -3;
        }
        delay(100); // long wait between AK8963 mode changes
        // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
        readAK8963Registers(AK8963_HXL,7,_buffer);
    }
    /* setting the sample rate divider */
    if(writeRegister(SMPDIV,srd) < 0){ // setting the sample rate divider
        std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
        return -4;
    }
    _srd = srd;
    return 1;
}

/* enables the data ready interrupt */
int MPU9250::enableDataReadyInterrupt() {
    // use low speed SPI for register setting
    _useSPIHS = false;
    /* setting the interrupt */
    if (writeRegister(INT_PIN_CFG,INT_PULSE_50US) < 0){ // setup interrupt, 50 us pulse
        return -1;
    }
    if (writeRegister(INT_ENABLE,INT_RAW_RDY_EN) < 0){ // set to data ready
        return -2;
    }
    return 1;
}

/* disables the data ready interrupt */
int MPU9250::disableDataReadyInterrupt() {
    // use low speed SPI for register setting
    _useSPIHS = false;
    if(writeRegister(INT_ENABLE,INT_DISABLE) < 0){ // disable interrupt
        return -1;
    }
    return 1;
}

/* configures and enables wake on motion, low power mode */
int MPU9250::enableWakeOnMotion(float womThresh_mg,LpAccelOdr odr) {
    // use low speed SPI for register setting
    _useSPIHS = false;
    // set AK8963 to Power Down
    writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
    // reset the MPU9250
    writeRegister(PWR_MGMNT_1,PWR_RESET);
    // wait for MPU-9250 to come back up
    delay(1);
    if(writeRegister(PWR_MGMNT_1,0x00) < 0){ // cycle 0, sleep 0, standby 0
        return -1;
    }
    if(writeRegister(PWR_MGMNT_2,DIS_GYRO) < 0){ // disable gyro measurements
        return -2;
    }
    if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ // setting accel bandwidth to 184Hz
        return -3;
    }
    if(writeRegister(INT_ENABLE,INT_WOM_EN) < 0){ // enabling interrupt to wake on motion
        return -4;
    }
    if(writeRegister(MOT_DETECT_CTRL,(ACCEL_INTEL_EN | ACCEL_INTEL_MODE)) < 0){ // enabling accel hardware intelligence
        return -5;
    }
    _womThreshold = remap(womThresh_mg, 0.0f, 1020.0f, 0.0f, 255.0f);
    if(writeRegister(WOM_THR,_womThreshold) < 0){ // setting wake on motion threshold
        return -6;
    }
    if(writeRegister(LP_ACCEL_ODR,(uint8_t)odr) < 0){ // set frequency of wakeup
        return -7;
    }
    if(writeRegister(PWR_MGMNT_1,PWR_CYCLE) < 0){ // switch to accel low power mode
        return -8;
    }
    return 1;
}

/* reads the most current data from MPU9250 and stores in buffer */
int MPU9250::readSensor() {
    _useSPIHS = true; // use the high speed SPI for data readout
    // grab the data from the MPU9250
    if (readRegisters(ACCEL_OUT, 21, _buffer) < 0) {
        return -1;
    }
    // combine into 16 bit values
    _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
    _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
    _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
    _tcounts  = (((int16_t)_buffer[6]) << 8) | _buffer[7];
    _gxcounts = (((int16_t)_buffer[8]) << 8) | _buffer[9];
    _gycounts = (((int16_t)_buffer[10]) << 8) | _buffer[11];
    _gzcounts = (((int16_t)_buffer[12]) << 8) | _buffer[13];
    _hxcounts = (((int16_t)_buffer[15]) << 8) | _buffer[14];
    _hycounts = (((int16_t)_buffer[17]) << 8) | _buffer[16];
    _hzcounts = (((int16_t)_buffer[19]) << 8) | _buffer[18];

    // transform and convert to float values
    _ax = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale) - _axb)*_axs;
    _ay = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale) - _ayb)*_ays;
    _az = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale) - _azb)*_azs;
    _gx = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb; 
    _gy = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb; 
    _gz = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
    _hx = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
    _hy = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
    _hz = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;
    _t = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
    return 1;
}

/* returns the accelerometer measurement in the x direction, m/s/s */
float MPU9250::getAccelX_mss() {
    return _ax;
}

/* returns the accelerometer measurement in the y direction, m/s/s */
float MPU9250::getAccelY_mss() {
    return _ay;
}

/* returns the accelerometer measurement in the z direction, m/s/s */
float MPU9250::getAccelZ_mss() {
    return _az;
}

/* returns the gyroscope measurement in the x direction, rad/s 2 points decimal presition */
float MPU9250::getGyroX_rads(bool filter) {
    if (filter) return pampas::round(_gyroOutputFilters[0].filter(_gx), 2);
    return pampas::round(_gx, 2);
}

/* returns the gyroscope measurement in the y direction, rad/s 2 points decimal presition */
float MPU9250::getGyroY_rads(bool filter) {
    if (filter) return pampas::round(_gyroOutputFilters[1].filter(_gy), 2);
    return pampas::round(_gy, 2);
}

/* returns the gyroscope measurement in the z direction, rad/s 2 points decimal presition */
float MPU9250::getGyroZ_rads(bool filter) {
    if (filter) return pampas::round(_gyroOutputFilters[2].filter(_gz), 2);
    return pampas::round(_gz, 2);
}

/* returns the magnetometer measurement in the x direction, uT */
float MPU9250::getMagX_uT() {
    return _hx;
}

/* returns the magnetometer measurement in the y direction, uT */
float MPU9250::getMagY_uT() {
    return _hy;
}

/* returns the magnetometer measurement in the z direction, uT */
float MPU9250::getMagZ_uT() {
    return _hz;
}

/* returns the die temperature, C */
float MPU9250::getTemperature_C() {
    return _t;
}



// GYRO STUFF

/* estimates the gyro biases */
int MPU9250::calibrateGyro(int durationSeconds) {
    // set the range, bandwidth, and srd
    if (setGyroRange(GYRO_RANGE_250DPS) < 0) {
        std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
        return -1;
    }
    if (setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0) {
        std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
        return -2;
    }
    if (setSrd(19) < 0) {
        std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
        return -3;
    }

    // take samples and find bias
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start + std::chrono::seconds(durationSeconds);

    // setea variables para modificarlas luego
    _gyroNumSamples = 0;
    _gxbD = 0.0;
    _gzbD = 0.0;
    _gybD = 0.0;
    setGyroBiasX_rads(0);
    setGyroBiasY_rads(0);
    setGyroBiasZ_rads(0);
    while (std::chrono::high_resolution_clock::now() < end) {
        _gyroNumSamples++;
        readSensor();
        _gxbD += getGyroX_rads();
        _gybD += getGyroY_rads();
        _gzbD += getGyroZ_rads();
        delay(20);
    }
    setGyroBiasX_rads(_gxbD / (float)_gyroNumSamples);
    setGyroBiasY_rads(_gybD / (float)_gyroNumSamples);
    setGyroBiasZ_rads(_gzbD / (float)_gyroNumSamples);

    std::cout<<__FILE__<<__LINE__<<"\t"<<_gxb <<"\t"<<_gyb <<"\t"<<_gzb<<"\t" ;

    // set the range, bandwidth, and srd back to what they were
    if (setGyroRange(_gyroRange) < 0) {
        return -4;
    }
    if (setDlpfBandwidth(_bandwidth) < 0) {
        return -5;
    }
    if (setSrd(_srd) < 0) {
        return -6;
    }
    return 1;
}

/* returns the gyro bias in the X direction, rad/s */
float MPU9250::getGyroBiasX_rads() {
    return _gxb;
}

/* returns the gyro bias in the Y direction, rad/s */
float MPU9250::getGyroBiasY_rads() {
    return _gyb;
}

/* returns the gyro bias in the Z direction, rad/s */
float MPU9250::getGyroBiasZ_rads() {
    return _gzb;
}

/* sets the gyro bias in the X direction to bias, rad/s */
void MPU9250::setGyroBiasX_rads(float bias) {
    _gxb = bias;
}

/* sets the gyro bias in the Y direction to bias, rad/s */
void MPU9250::setGyroBiasY_rads(float bias) {
    _gyb = bias;
}

/* sets the gyro bias in the Z direction to bias, rad/s */
void MPU9250::setGyroBiasZ_rads(float bias) {
    _gzb = bias;
}


//ACC STUFF

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int MPU9250::calibrateAccel() {
    // set the range, bandwidth, and srd
    if (setAccelRange(ACCEL_RANGE_2G) < 0) {
        return -1;
    }
    if (setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0) {
        return -2;
    }
    if (setSrd(19) < 0) {
        return -3;
    }

    // take samples and find min / max
    _axbD = 0;
    _aybD = 0;
    _azbD = 0;

    for (size_t i=0; i < _numSamples; i++) {
        readSensor();
        _axbD += (getAccelX_mss()/_axs + _axb)/((double)_numSamples);
        _aybD += (getAccelY_mss()/_ays + _ayb)/((double)_numSamples);
        _azbD += (getAccelZ_mss()/_azs + _azb)/((double)_numSamples);
        std::cout<<"\t"<<_axbD <<"\t"<<_aybD <<"\t"<<_azbD<<"\n" ;
        delay(20);
    }
    if (_axbD > 9.0f) {
        _axmax = (float)_axbD;
    }
    if (_aybD > 9.0f) {
        _aymax = (float)_aybD;
    }
    if (_azbD > 9.0f) {
        _azmax = (float)_azbD;
    }
    if (_axbD < -9.0f) {
        _axmin = (float)_axbD;
    }
    if (_aybD < -9.0f) {
        _aymin = (float)_aybD;
    }
    if (_azbD < -9.0f) {
        _azmin = (float)_azbD;
    }

    // find bias and scale factor
    if ((abs(_axmin) > 9.0f) && (abs(_axmax) > 9.0f)) {
        _axb = (_axmin + _axmax) / 2.0f;
        _axs = G/((abs(_axmin) + abs(_axmax)) / 2.0f);
    }
    if ((abs(_aymin) > 9.0f) && (abs(_aymax) > 9.0f)) {
        _ayb = (_aymin + _aymax) / 2.0f;
        _ays = G/((abs(_aymin) + abs(_aymax)) / 2.0f);
    }
    if ((abs(_azmin) > 9.0f) && (abs(_azmax) > 9.0f)) {
        _azb = (_azmin + _azmax) / 2.0f;
        _azs = G/((abs(_azmin) + abs(_azmax)) / 2.0f);
    }
    std::cout<<__FILE__<<__LINE__<<"\t"<<_axb <<"\t"<<_ayb <<"\t"<<_azb<<"\t" ;

    // set the range, bandwidth, and srd back to what they were
    if (setAccelRange(_accelRange) < 0) {
        return -4;
    }
    if (setDlpfBandwidth(_bandwidth) < 0) {
        return -5;
    }
    if (setSrd(_srd) < 0) {
        return -6;
    }
    return 1;
}

int MPU9250::calibrateAccel2() {
    // set the range, bandwidth, and srd
    if (setAccelRange(ACCEL_RANGE_2G) < 0) {
        return -1;
    }
    if (setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0) {
        return -2;
    }
    if (setSrd(19) < 0) {
        return -3;
    }

    // take samples and find min / max
    _axbD = 0;
    _aybD = 0;
    _azbD = 0;

    for (size_t i=0; i < _numSamples; i++) {
        readSensor();
        _axbD += getAccelX_mss();
        _aybD += getAccelY_mss();
        _azbD += getAccelZ_mss();
        delay(20);
    }
    
    setAccelCalX(_axbD / _numSamples, 1.0f);
    setAccelCalY(_aybD / _numSamples, 1.0f);
    setAccelCalZ(_azbD / _numSamples, 1.0f);

    std::cout<<__FILE__<<__LINE__<<"\t"<<_axb <<"\t"<<_ayb <<"\t"<<_azb<<"\t" ;

    // set the range, bandwidth, and srd back to what they were
    if (setAccelRange(_accelRange) < 0) {
        return -4;
    }
    if (setDlpfBandwidth(_bandwidth) < 0) {
        return -5;
    }
    if (setSrd(_srd) < 0) {
        return -6;
    }
    return 1;
}

/* returns the accelerometer bias in the X direction, m/s/s */
float MPU9250::getAccelBiasX_mss() {
    return _axb;
}

/* returns the accelerometer scale factor in the X direction */
float MPU9250::getAccelScaleFactorX() {
    return _axs;
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float MPU9250::getAccelBiasY_mss() {
    return _ayb;
}

/* returns the accelerometer scale factor in the Y direction */
float MPU9250::getAccelScaleFactorY() {
    return _ays;
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float MPU9250::getAccelBiasZ_mss() {
    return _azb;
}

/* returns the accelerometer scale factor in the Z direction */
float MPU9250::getAccelScaleFactorZ() {
    return _azs;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void MPU9250::setAccelCalX(float bias,float scaleFactor) {
    _axb = bias;
    _axs = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void MPU9250::setAccelCalY(float bias,float scaleFactor) {
    _ayb = bias;
    _ays = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void MPU9250::setAccelCalZ(float bias,float scaleFactor) {
    _azb = bias;
    _azs = scaleFactor;
}


// MAG STUFF

/* finds bias and scale factor calibration for the magnetometer,
the sensor should be rotated in a figure 8 motion until complete */
int MPU9250::calibrateMag() {
    // set the srd
    if (setSrd(19) < 0) {
        return -1;
    }

    // get a starting set of data
    readSensor();
    _hxmax = getMagX_uT();
    _hxmin = getMagX_uT();
    _hymax = getMagY_uT();
    _hymin = getMagY_uT();
    _hzmax = getMagZ_uT();
    _hzmin = getMagZ_uT();

    // collect data to find max / min in each channel
    _counter = 0;
    while (_counter < _maxCounts) {
        _delta = 0.0f;
        _framedelta = 0.0f;
        readSensor();
        _hxfilt = (_hxfilt*((float)_coeff-1)+(getMagX_uT()/_hxs+_hxb))/((float)_coeff);
        _hyfilt = (_hyfilt*((float)_coeff-1)+(getMagY_uT()/_hys+_hyb))/((float)_coeff);
        _hzfilt = (_hzfilt*((float)_coeff-1)+(getMagZ_uT()/_hzs+_hzb))/((float)_coeff);
        if (_hxfilt > _hxmax) {
            _delta = _hxfilt - _hxmax;
            _hxmax = _hxfilt;
        }
        if (_delta > _framedelta) {
            _framedelta = _delta;
        }
        if (_hyfilt > _hymax) {
            _delta = _hyfilt - _hymax;
            _hymax = _hyfilt;
        }
        if (_delta > _framedelta) {
            _framedelta = _delta;
        }
        if (_hzfilt > _hzmax) {
            _delta = _hzfilt - _hzmax;
            _hzmax = _hzfilt;
        }
        if (_delta > _framedelta) {
            _framedelta = _delta;
        }
        if (_hxfilt < _hxmin) {
            _delta = abs(_hxfilt - _hxmin);
            _hxmin = _hxfilt;
        }
        if (_delta > _framedelta) {
            _framedelta = _delta;
        }
        if (_hyfilt < _hymin) {
            _delta = abs(_hyfilt - _hymin);
            _hymin = _hyfilt;
        }
        if (_delta > _framedelta) {
            _framedelta = _delta;
        }
        if (_hzfilt < _hzmin) {
            _delta = abs(_hzfilt - _hzmin);
            _hzmin = _hzfilt;
        }
        if (_delta > _framedelta) {
            _framedelta = _delta;
        }
        if (_framedelta > _deltaThresh) {
            _counter = 0;
        } else {
            _counter++;
        }
        delay(20);
    }

    // find the magnetometer bias
    _hxb = (_hxmax + _hxmin) / 2.0f;
    _hyb = (_hymax + _hymin) / 2.0f;
    _hzb = (_hzmax + _hzmin) / 2.0f;
    std::cout<<__FILE__<<__LINE__<<"\t"<<_hxb <<"\t"<<_hyb <<"\t"<<_hzb<<"\t" ;

    // find the magnetometer scale factor
    _hxs = (_hxmax - _hxmin) / 2.0f;
    _hys = (_hymax - _hymin) / 2.0f;
    _hzs = (_hzmax - _hzmin) / 2.0f;
    _avgs = (_hxs + _hys + _hzs) / 3.0f;
    _hxs = _avgs/_hxs;
    _hys = _avgs/_hys;
    _hzs = _avgs/_hzs;

    // set the srd back to what it was
    if (setSrd(_srd) < 0) {
        return -2;
    }
    return 1;
}

/* returns the magnetometer bias in the X direction, uT */
float MPU9250::getMagBiasX_uT() {
    return _hxb;
}

/* returns the magnetometer scale factor in the X direction */
float MPU9250::getMagScaleFactorX() {
    return _hxs;
}

/* returns the magnetometer bias in the Y direction, uT */
float MPU9250::getMagBiasY_uT() {
    return _hyb;
}

/* returns the magnetometer scale factor in the Y direction */
float MPU9250::getMagScaleFactorY() {
    return _hys;
}

/* returns the magnetometer bias in the Z direction, uT */
float MPU9250::getMagBiasZ_uT() {
    return _hzb;
}

/* returns the magnetometer scale factor in the Z direction */
float MPU9250::getMagScaleFactorZ() {
    return _hzs;
}

/* sets the magnetometer bias (uT) and scale factor in the X direction */
void MPU9250::setMagCalX(float bias,float scaleFactor) {
    _hxb = bias;
    _hxs = scaleFactor;
}

/* sets the magnetometer bias (uT) and scale factor in the Y direction */
void MPU9250::setMagCalY(float bias,float scaleFactor) {
    _hyb = bias;
    _hys = scaleFactor;
}

/* sets the magnetometer bias (uT) and scale factor in the Z direction */
void MPU9250::setMagCalZ(float bias,float scaleFactor) {
    _hzb = bias;
    _hzs = scaleFactor;
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
int MPU9250::whoAmI(){
    // read the WHO AM I register
    if (readRegisters(WHO_AM_I,1,_buffer) < 0) {
        return -1;
    }
    // return the register value
    return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
int MPU9250::whoAmIAK8963(){
    // read the WHO AM I register
    if (readAK8963Registers(AK8963_WHO_AM_I,1,_buffer) < 0) {
        return -1;
    }
    // return the register value
    return _buffer[0];
}

/* writes a register to the AK8963 given a register address and data */
int MPU9250::writeAK8963Register(uint8_t subAddress, uint8_t data){
    // set slave 0 to the AK8963 and set for write
    if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR) < 0) {
        std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
        return -1;
    }
    // set the register to the desired AK8963 sub address
    if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
        std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
        return -2;
    }
    // store the data for write
    if (writeRegister(I2C_SLV0_DO,data) < 0) {
        std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
        return -3;
    }
    // enable I2C and send 1 byte
    if (writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1) < 0) {
        std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
        return -4;
    }

    // read the register and confirm
    if (readAK8963Registers(subAddress,1,_buffer) < 0) {
        std::cerr<<__FILE__<<__LINE__<<": error."<<std::endl;
        return -5;
    }

    if(_buffer[0] == data) {
        return 1;
    } else{
        return -6;
    }
}

/* reads registers from the AK8963 */
int MPU9250::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){
    // set slave 0 to the AK8963 and set for read
    if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG) < 0) {
        return -1;
    }
    // set the register to the desired AK8963 sub address
    if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
        return -2;
    }
    // enable I2C and request the bytes
    if (writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count) < 0) {
        return -3;
    }
    delay(1); // takes some time for these registers to fill
    // read the bytes off the MPU9250 EXT_SENS_DATA registers
    _status = readRegisters(EXT_SENS_DATA_00,count,dest);
    return _status;
}

/* calculates pitch and roll */
int MPU9250::updateAngles() {

    // start gyro clock for angle measurement
    if (!_gyroTimerStarted) clock_gettime(CLOCK_REALTIME, &_gyroPrevTime);
    _gyroTimerStarted = true;
    
    // Obtener tiempo transcurrido en segundos (dt)
    clock_gettime(CLOCK_REALTIME, &_gyroCurrentTime);
    float dt = (_gyroCurrentTime.tv_sec - _gyroPrevTime.tv_sec) + 
               (_gyroCurrentTime.tv_nsec - _gyroPrevTime.tv_nsec) / 1e9;
    
    // Actualizar ángulos usando la integración
    this->_thetaX += getGyroX_rads(true) * dt;
    this->_thetaY += getGyroY_rads(true) * dt;

    // Guardar el tiempo actual como referencia para la siguiente iteración
    _gyroPrevTime = _gyroCurrentTime;

    return 1;
}

/* returns angle made from X axis */
float MPU9250::getThetaX() {
    return pampas::round(this->_thetaX * (180.0 / M_PI), 2);
}

/* returns angle made from Y axis */
float MPU9250::getThetaY() {
    return pampas::round(this->_thetaY * (180.0 / M_PI), 2);
}


/* configures and enables the FIFO buffer  */
int MPU9250FIFO::enableFifo(bool accel,bool gyro,bool mag,bool temp) {
    // use low speed SPI for register setting
    _useSPIHS = false;
    if(writeRegister(USER_CTRL, (0x40 | I2C_MST_EN)) < 0){
        return -1;
    }
    if(writeRegister(FIFO_EN,(accel*FIFO_ACCEL)|(gyro*FIFO_GYRO)|(mag*FIFO_MAG)|(temp*FIFO_TEMP)) < 0){
        return -2;
    }
    _enFifoAccel = accel;
    _enFifoGyro = gyro;
    _enFifoMag = mag;
    _enFifoTemp = temp;
    _fifoFrameSize = accel*6 + gyro*6 + mag*7 + temp*2;
    return 1;
}

/* reads data from the MPU9250 FIFO and stores in buffer */
int MPU9250FIFO::readFifo() {
    _useSPIHS = true; // use the high speed SPI for data readout
    // get the fifo size
    readRegisters(FIFO_COUNT, 2, _buffer);
    _fifoSize = (((uint16_t) (_buffer[0]&0x0F)) <<8) + (((uint16_t) _buffer[1]));
    // read and parse the buffer
    for (size_t i=0; i < _fifoSize/_fifoFrameSize; i++) {
        // grab the data from the MPU9250
        if (readRegisters(FIFO_READ,_fifoFrameSize,_buffer) < 0) {
            return -1;
        }
        if (_enFifoAccel) {
            // combine into 16 bit values
            _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];
            _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
            _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
            // transform and convert to float values
            _axFifo[i] = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale)-_axb)*_axs;
            _ayFifo[i] = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale)-_ayb)*_ays;
            _azFifo[i] = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale)-_azb)*_azs;
            _aSize = _fifoSize/_fifoFrameSize;
        }
        if (_enFifoTemp) {
            // combine into 16 bit values
            _tcounts = (((int16_t)_buffer[0 + _enFifoAccel*6]) << 8) | _buffer[1 + _enFifoAccel*6];
            // transform and convert to float values
            _tFifo[i] = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
            _tSize = _fifoSize/_fifoFrameSize;
        }
        if (_enFifoGyro) {
            // combine into 16 bit values
            _gxcounts = (((int16_t)_buffer[0 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[1 + _enFifoAccel*6 + _enFifoTemp*2];
            _gycounts = (((int16_t)_buffer[2 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[3 + _enFifoAccel*6 + _enFifoTemp*2];
            _gzcounts = (((int16_t)_buffer[4 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[5 + _enFifoAccel*6 + _enFifoTemp*2];
            // transform and convert to float values
            _gxFifo[i] = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
            _gyFifo[i] = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
            _gzFifo[i] = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
            _gSize = _fifoSize/_fifoFrameSize;
        }
        if (_enFifoMag) {
            // combine into 16 bit values
            _hxcounts = (((int16_t)_buffer[1 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[0 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
            _hycounts = (((int16_t)_buffer[3 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[2 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
            _hzcounts = (((int16_t)_buffer[5 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[4 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
            // transform and convert to float values
            _hxFifo[i] = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
            _hyFifo[i] = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
            _hzFifo[i] = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;
            _hSize = _fifoSize/_fifoFrameSize;
        }
    }
    return 1;
}

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void MPU9250FIFO::getFifoAccelX_mss(size_t *size,float* data) {
    *size = _aSize;
    memcpy(data,_axFifo,_aSize*sizeof(float));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void MPU9250FIFO::getFifoAccelY_mss(size_t *size,float* data) {
    *size = _aSize;
    memcpy(data,_ayFifo,_aSize*sizeof(float));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void MPU9250FIFO::getFifoAccelZ_mss(size_t *size,float* data) {
    *size = _aSize;
    memcpy(data,_azFifo,_aSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the x direction, rad/s */
void MPU9250FIFO::getFifoGyroX_rads(size_t *size,float* data) {
    *size = _gSize;
    memcpy(data,_gxFifo,_gSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the y direction, rad/s */
void MPU9250FIFO::getFifoGyroY_rads(size_t *size,float* data) {
    *size = _gSize;
    memcpy(data,_gyFifo,_gSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the z direction, rad/s */
void MPU9250FIFO::getFifoGyroZ_rads(size_t *size,float* data) {
    *size = _gSize;
    memcpy(data,_gzFifo,_gSize*sizeof(float));
}

/* returns the magnetometer FIFO size and data in the x direction, uT */
void MPU9250FIFO::getFifoMagX_uT(size_t *size,float* data) {
    *size = _hSize;
    memcpy(data,_hxFifo,_hSize*sizeof(float));
}

/* returns the magnetometer FIFO size and data in the y direction, uT */
void MPU9250FIFO::getFifoMagY_uT(size_t *size,float* data) {
    *size = _hSize;
    memcpy(data,_hyFifo,_hSize*sizeof(float));
}

/* returns the magnetometer FIFO size and data in the z direction, uT */
void MPU9250FIFO::getFifoMagZ_uT(size_t *size,float* data) {
    *size = _hSize;
    memcpy(data,_hzFifo,_hSize*sizeof(float));
}

/* returns the die temperature FIFO size and data, C */
void MPU9250FIFO::getFifoTemperature_C(size_t *size,float* data) {
    *size = _tSize;
    memcpy(data,_tFifo,_tSize*sizeof(float));
}



// void  delay(int t){
//     while(t--){
//         usleep(1000);
//     }
// }

// long map(long x, long in_min, long in_max, long out_min, long out_max){
//     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }
}


