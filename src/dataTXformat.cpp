// #include "Arduino.h"
// #include <SoftwareSerial.h>
// #include <TinyGPS++.h>
// #include <Wire.h>
// #include <SPI.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BME280.h>

// #define SEALEVELPRESSURE_HPA (1013.25)
// TinyGPSPlus gps;
// Adafruit_BME280 bme; // I2C
// SoftwareSerial ss(8,7); // RX TX

// float latitude = 0.0f, longitude = 0.0f, gpsAltitude = 0.0f;
// float altitudeMeter = 0.0f, tempCelc = 0.0f, presshPa = 0.0f, humidity = 0.0f;

// void getGPSInfo();
// void sendTelemetry();
// void calculateSensors();

// void setup(){
//     Serial.begin(115200);
//     ss.begin(115200);
//    // while(!Serial);    // time to get serial running BURASI SIKICI SIKINTILI OLABILIR!!!!!!!!

//     // default settings
//     bme.begin(0x76);  
// }

// void loop() {
//     while(ss.available() > 0)
//       if(gps.encode(ss.read()))
//         getGPSInfo();
//     calculateSensors();
//     sendTelemetry();
// }
// void getGPSInfo()
// { 
//   if (gps.location.isValid())
//   {
//     latitude = gps.location.lat();
//     longitude = gps.location.lng();
//   }
//   else
//   {
//     latitude = 0.0000000f;
//     longitude = 0.0000000f;
//   }
//   if(gps.altitude.isValid()){
//     gpsAltitude = gps.altitude.meters();
//   }
//   else
//     gpsAltitude = 0.00000f;
// }
// void sendTelemetry() {
//     Serial.print("Orion,C,"); Serial.print(altitudeMeter,2); Serial.print(","); Serial.print(gpsAltitude,6); Serial.print(",");
//     Serial.print(latitude, 6); Serial.print(","); Serial.print(longitude,6); Serial.print(","); Serial.print(tempCelc,2);
//     Serial.print(","); Serial.print(presshPa,2); Serial.print(","); Serial.print(humidity,2); Serial.println(",Orion");
// }

// void calculateSensors() {
//     tempCelc = bme.readTemperature();
//     presshPa = bme.readPressure() / 100.0f;
//     altitudeMeter = bme.readAltitude(SEALEVELPRESSURE_HPA);
//     humidity = bme.readHumidity();
// }