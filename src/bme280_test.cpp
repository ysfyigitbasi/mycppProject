// #include "Arduino.h"
// #include "BME280.h"

// float Temperature, Pressure, Humidity;
// BME280 bme;

// void setup(){
//     Serial.begin(9600);
//     Wire.begin();
//     bme.BME280_Config(OSRS_4, OSRS_4, OSRS_4, MODE_NORMAL, T_SB_10, IIR_4);
// }

// void loop() {
//     bme.BME280_Measure(&Temperature, &Pressure, &Humidity);
//     // sprintf(x,"Temp %.2f, Pressure %.2f, Humidity %.2f",(double)Temperature,(double)Pressure,(double)Humidity);
//     Serial.print(Temperature); Serial.print(","); Serial.print(Pressure); Serial.print(","); Serial.println(Humidity);
// }
