#include "bmp180.h"

float BMP180::getAltitude(){
    getPressure();
    SKF_baseAltitude = 44330 * (1 - pow((((double) SKF_pressure) / 101325.0), us));
    return SKF_baseAltitude;
}

float BMP180::getTemperature() {
    readUT();
    x1 = (((temperatureCount - (long) ac6) * (long) ac5) >> 15);
    x2 = ((long) mc << 11) / (x1 + md);
    b5 = x1 + x2;
    temperature = (b5 + 8) >> 4;
    return temperature;
}

void BMP180::getPressure() {
    long pressure;
    readUP();
    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = ac2 * b6 >> 11;
    x3 = x1 + x2;
    b3 = (((((long) ac1) * 4 + x3) << OSS) + 2) >> 2;
    x1 = ac3 * b6 >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
    b7 = ((unsigned long) (pressureCount - b3) * (50000 >> OSS));
    if (b7 < 0x80000000)
        pressure = (b7 << 1) / b4;
    else
        pressure = (b7 / b4) << 1;
    x1 = (pressure >> 8) * (pressure >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * pressure) >> 16;
    pressure += (x1 + x2 + 3791) >> 4;
    SKF_pressure = pressurePA.updateEstimateLong(pressure);
}

void BMP180::readUT() {
    writeByte(BMP180_REG_CONTROL,BMP180_CMD_MEASURE_TEMP);
    delay(5);
    pokeAddress(BMP180_REG_DATA);
    Wire.requestFrom(BMP180_ADDRESS, 2);
    while (Wire.available() < 2);
    temperatureCount = Wire.read() << 8 | Wire.read();    
}

void BMP180::readUP() {
    static uint8_t msb, lsb, xlsb;
    writeByte(BMP180_REG_CONTROL, (BMP180_CMD_MEASURE_PRESSURE + (OSS << 6)));
    delay(2+(3<<OSS));
    pokeAddress(BMP180_REG_DATA);
    Wire.requestFrom(BMP180_ADDRESS, 3);
    while(Wire.available() < 3);
    msb = Wire.read();
    lsb = Wire.read();
    xlsb = Wire.read();
    pressureCount = (((long int) msb << 16) | ((long int) lsb << 8) | (long int) xlsb) >> (8-OSS);
}


void BMP180::start(Sampling sample){
    OSS = sample;
    readRegisters();
}

void BMP180::readRegisters(){
    ac1 = readBytes(BMP180_REG_AC1);
    ac2 = readBytes(BMP180_REG_AC2);
    ac3 = readBytes(BMP180_REG_AC3);
    ac4 = (unsigned short) readBytes(BMP180_REG_AC4);
    ac5 = (unsigned short) readBytes(BMP180_REG_AC5);
    ac6 = (unsigned short) readBytes(BMP180_REG_AC6);
    b1 = readBytes(BMP180_REG_B1);
    b2 = readBytes(BMP180_REG_B2);
    mb = readBytes(BMP180_REG_MB);
    mc = readBytes(BMP180_REG_MC);
    md = readBytes(BMP180_REG_MD);
}

void BMP180::writeByte(uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(BMP180_ADDRESS);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

void BMP180::pokeAddress(uint8_t subAddress)
{
  Wire.beginTransmission(BMP180_ADDRESS);         // Initialize the Tx buffer
  Wire.write(subAddress);	                 // Put slave register address in Tx buffer
  Wire.endTransmission();             // Send the Tx buffer, but send a restart to keep connection alive
}

short BMP180::readBytes(uint8_t subAddress)
{
  Wire.beginTransmission(BMP180_ADDRESS);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(BMP180_ADDRESS, 2);  // Read bytes from slave register address
  while (Wire.available() < 2);
  return (short int) Wire.read() << 8 | Wire.read(); // msb << lsb
}