#include "Arduino.h"
#include "Wire.h"
#ifndef bmp180_h
#define bmp180_h

#define BMP180_ADDRESS                (0x77)

// COMPUTATION VARIABLE REGISTERS
#define BMP180_REG_AC1                (0xAA)
#define BMP180_REG_AC2                (0xAC)
#define BMP180_REG_AC3                (0xAE)
#define BMP180_REG_AC4                (0xB0)
#define BMP180_REG_AC5                (0xB2)
#define BMP180_REG_AC6                (0xB4)
#define BMP180_REG_B1                 (0xB6)
#define BMP180_REG_B2                 (0xB8)
#define BMP180_REG_MB                 (0xBA)
#define BMP180_REG_MC                 (0xBC)
#define BMP180_REG_MD                 (0xBE)

#define BMP180_REG_CONTROL            (0xF4)
#define BMP180_REG_DATA               (0xF6)

#define BMP180_CMD_MEASURE_TEMP       (0x2E) // Max conversion time 4.5ms
#define BMP180_CMD_MEASURE_PRESSURE   (0x34) // Max conversion time 4.5ms (OSS = 0)

enum Sampling{
    ULT_LOW_PWR_4p5ms = 0,
    STANDART_7p5ms,
    HIGH_RES_13p5ms,
    ULT_HIGH_RES
};
#include "SimpleKalmanFilter.h"

class BMP180 {
    SimpleKalmanFilter pressurePA = SimpleKalmanFilter(8.0f,8.0f,0.01f);
    
    short int ac1, ac2, ac3, b1, b2, mb, mc, md;
    unsigned short int ac4, ac5, ac6;
    // DYNAMIC VARIABLES//
    long x1, x2, x3, b3, b5, b6, pressureCount, temperatureCount, SKF_pressure;
    unsigned long int b4, b7;

    float temperature, SKF_baseAltitude, altitude;
    int OSS = ULT_LOW_PWR_4p5ms;
    const double us = 1.0/5.255;

    void writeByte(uint8_t subAddress, uint8_t data);
    void pokeAddress(uint8_t subAddress);
    short readBytes(uint8_t subAddress);
    void getPressure();
    void readRegisters();
    void readUT();
    void readUP();
public:

    void start(Sampling sample);
    float getTemperature();
    
    float getAltitude();
};


#endif
