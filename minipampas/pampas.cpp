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
    gpio::pinMode(pin, PWM_OUTPUT);
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

namespace pampas
{

template <typename T>
T remap(T value, T in_min, T in_max, T out_min, T out_max) {
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

void delay(int ms) {
	::delay(ms);  
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


void LowPass::setAlpha(double value) 
{
    this->alpha = value;
}


double LowPass::filter(double input) {
    double output = this->alpha * input + (1.0 - this->alpha) * this->prevOutput;
    prevOutput = output;
    return output;
}

}
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
    this->distance += this->wheelCircumference;

    clock_gettime(CLOCK_MONOTONIC, &this->endTime);
    this->timeInterval = (endTime.tv_sec - startTime.tv_sec) +
                   (endTime.tv_nsec - startTime.tv_nsec) / 1e9;
    this->speed = (timeInterval > 0) ? (this->wheelCircumference / this->timeInterval) : 0.0;
    clock_gettime(CLOCK_MONOTONIC, &this->startTime);
    
    this->udpated = true;    
}
 
void Velocimeter::start() 
{
    this->udpated = false;
    if (this->started == true) return;

    gpio::pinMode(this->pin, INPUT);
    gpio::onInterrupt(this->pin, INT_EDGE_RISING, &pulseHandlerWrapper);
    clock_gettime(CLOCK_MONOTONIC, &this->startTime);
    
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
    integrator = 0.0f;
    prevError = 0.0f;
    differentiator = 0.0f;
    prevMeasurement = 0.0f;
    out = 0.0f;
}

double PID::calculate(double setpoint, double measurement, double sampleTime) 
{
    if (!gainsDefined) throw std::invalid_argument( "Faltan definir las ganancias del controlador PID" );
    if (!paramsDefined) throw std::invalid_argument( "Faltan definir los parametros del controlador PID" );

    double error = setpoint - measurement;
    double proportional = Kp * error;
    integrator += 0.5f * Ki * sampleTime * (error + prevError);

    if (integrator > maxOutputInt) {
        integrator = maxOutputInt;
    } else if (integrator < minOutputInt) {
        integrator = minOutputInt;
    }

    // Derivativo (filtro de paso bajo)
    differentiator = -(2.0f * Kd * (measurement - prevMeasurement) +
                       (2.0f * tau - sampleTime) * differentiator) /
                     (2.0f * tau + sampleTime);

    out = proportional + integrator + differentiator;

    if (out > maxOutput) {
        out = maxOutput;
    } else if (out < minOutput) {
        out = minOutput;
    }

    prevError = error;
    prevMeasurement = measurement;

    return out;
}

void PID::setGains(double kp, double ki, double kd) 
{
    this->gainsDefined = true;
    this->Kp = kp;
    this->Ki = ki;
    this->Kd = kd;
}

void PID::setParameters(double tau, double minOutput, double maxOutput, double minOutputInt, double maxOutputInt) 
{
    this->paramsDefined = true;
    this->tau = tau;
    this->minOutput = minOutput;
    this->maxOutput = maxOutput;
    this->minOutputInt = minOutputInt;
    this->maxOutputInt = maxOutputInt;
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



void Drive::controlledSpeed(double speed) 
{
    while (this->running) {

        this->velocimeter.start();
        this->velocimeter.waitForUpdate();

        // Calcular velocidad utilizando PID
        double newSpeed = this->pid.calculate(
            speed, 
            this->velocimeter.getSpeed(), 
            this->velocimeter.getUpdateTimeInterval()
        );

        this->motor.setPulseWidth(this->MsToPulseWidth.convert(newSpeed));
    }
}

void Drive::run(double speed) 
{
    if (speed == this->runningSpeed) return; // evita que se cree un nuevo thread para una velocidad ya definida

    this->stop();
    this->runningSpeed = speed;
    this->running = true;
    
    this->control = std::thread(&Drive::controlledSpeed, this, speed);
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
#ifdef USING_VSCODE_AS_EDITOR
    #include "MPU9250.h"
#endif

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

namespace pampas
{

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




    
bool MPU9250::handleCalibration(bool overwriteCalibrationFile) {

    if (calibrationFileExists() && !overwriteCalibrationFile) {
        std::cout << "Archivo de calibracion encontrado. Cargando valores..." << std::endl;
        if (!loadCalibration()) {
            std::cerr << "Error: No se pudo cargar el archivo de calibracion." << std::endl;
            return false;
        }
        std::cout << "Calibracion cargada con exito." << std::endl;
        return true;
    } else if (!calibrationFileExists() && !overwriteCalibrationFile) {
        std::cout << "Error: Archivo de calibracion no encontrado. Calibracion cancelada." << std::endl; 
        return false;
    }

    // En caso de param: overwrite=true
    if (!calibrate()) {
        std::cerr << "Error: Fallo la calibracion de los sensores." << std::endl;
        return false;
    }

    std::cout << "Guardando valores de calibracion en " << calibration_filename << "..." << std::endl;
    if (!saveCalibration()) {
        std::cerr << "Error: No se pudo guardar el archivo de calibracion." << std::endl;
        return false;
    }

    if (!loadCalibration()) {
        std::cerr << "Error: No se pudo cargar el archivo de calibracion." << std::endl;
        return false;
    }
    std::cout << "Calibracion completada y guardada con éxito." << std::endl;
    return true;
}


bool MPU9250::calibrate() {
    std::cout << "Calibrando giroscopio..." << std::endl;
    if (calibrateGyro() < 0) {
        std::cerr << "Error: La calibracion del giroscopio fallo." << std::endl;
        return false;
    }

    std::cout << "Calibrando acelerometro..." << std::endl;
    if (calibrateAccel() < 0) {
        std::cerr << "Error: La calibracion del acelerometro fallo." << std::endl;
        return false;
    }

    std::cout << "Calibrando magnetometro. Realiza movimientos en figura de 8..." << std::endl;
    if (calibrateMag() < 0) {
        std::cerr << "Error: La calibracion del magnetometro fallo." << std::endl;
        return false;
    }

    std::cout << "Calibracion completada para todos los sensores." << std::endl;
    return true;
}

bool MPU9250::calibrationFileExists() {
    std::ifstream file(calibration_filename);
    return file.good(); 
}

bool MPU9250::saveCalibration() {
    Writer writer(calibration_filename, "Sensor,Bias,Scale");

    writer.write_row({"Gyro_X", std::to_string(_gxb), "1.0"});
    writer.write_row({"Gyro_Y", std::to_string(_gyb), "1.0"});
    writer.write_row({"Gyro_Z", std::to_string(_gzb), "1.0"});

    writer.write_row({"Accel_X", std::to_string(_axb), std::to_string(_axs)});
    writer.write_row({"Accel_Y", std::to_string(_ayb), std::to_string(_ays)});
    writer.write_row({"Accel_Z", std::to_string(_azb), std::to_string(_azs)});

    writer.write_row({"Mag_X", std::to_string(_hxb), std::to_string(_hxs)});
    writer.write_row({"Mag_Y", std::to_string(_hyb), std::to_string(_hys)});
    writer.write_row({"Mag_Z", std::to_string(_hzb), std::to_string(_hzs)});

    writer.close();
    return true;
}

bool MPU9250::loadCalibration() {
    std::ifstream file(calibration_filename);
    if (!file.is_open()) {
        std::cerr << "Error: No se pudo abrir el archivo para cargar la calibracion." << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string sensor, bias_str, scale_str;
        if (std::getline(ss, sensor, ',') && std::getline(ss, bias_str, ',') && std::getline(ss, scale_str, ',')) {
            float bias = std::stof(bias_str);
            float scale = std::stof(scale_str);

            if (sensor == "Gyro_X") {
                _gxb = bias;
            } else if (sensor == "Gyro_Y") {
                _gyb = bias;
            } else if (sensor == "Gyro_Z") {
                _gzb = bias;
            } else if (sensor == "Accel_X") {
                _axb = bias;
                _axs = scale;
            } else if (sensor == "Accel_Y") {
                _ayb = bias;
                _ays = scale;
            } else if (sensor == "Accel_Z") {
                _azb = bias;
                _azs = scale;
            } else if (sensor == "Mag_X") {
                _hxb = bias;
                _hxs = scale;
            } else if (sensor == "Mag_Y") {
                _hyb = bias;
                _hys = scale;
            } else if (sensor == "Mag_Z") {
                _hzb = bias;
                _hzs = scale;
            }
        }
    }

    file.close();
    return true;
}



/* MPU9250 object, input the I2C bus and address */
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
    std::cout<<__FILE__<<__LINE__<<"  "<<_magScaleX<<"\t"<<_magScaleY<<"\t"<<_magScaleZ<<"\n";
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
    if (calibrateGyro() < 0) {
        std::cerr<<__FILE__<<__LINE__<<": error calibrateGyro."<<std::endl;
        return -21;
    }

/*
    if (calibrateAccel() < 0) {
        std::cerr<<__FILE__<<__LINE__<<": error calibrateAccel."<<std::endl;
        return -20;
    }

    if (calibrateMag() < 0) {
        std::cerr<<__FILE__<<__LINE__<<": error calibrateMag."<<std::endl;
        return -22;
    }
 */
    // successful init, return 1
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

}
