// #include "MPU6050.h"
// // #include "bmp180.h"
// #include "SimpleKalmanFilter.h"

// // #include <SoftwareSerial.h>
// // #include <TinyGPS++.h>

// MPU6050lib mpu;
// // BMP180 bmp;
// // SoftwareSerial ss(8,7); // RX TX
// // TinyGPSPlus gps;

// enum FLIGHT_STATE{
//   ON_THE_RAMP = 0,
//   TAKE_OFF = 1,
//   FIRST_SEPERATION = 2,
//   SECOND_SEPERATION = 3,
//   GROUND = 4
// };

// SimpleKalmanFilter skfPitch = SimpleKalmanFilter(1.0f, 1.0f, 0.016f);
// SimpleKalmanFilter skfRoll = SimpleKalmanFilter(1.0f, 1.0f, 0.016f);
// SimpleKalmanFilter skfYaw = SimpleKalmanFilter(1.0f, 1.0f, 0.016f);
// SimpleKalmanFilter skfAz = SimpleKalmanFilter(0.1f, 0.1f, 0.2f);
// uint32_t delt_t = 0;    // used to control display output rate
// uint32_t lastUpdate = 0;// used to calculate integration interval
// uint32_t Now = 0;       // used to calculate integration interval

// float pitch, yaw, roll; // FLIGHT EULER'S ANGLE
// float rakim, temperature;
// float SKF_pitch, SKF_az, SKF_roll, SKF_yaw;

// float latitude = 0.0f, longitude = 0.0f, gpsAltitude = 0.0f;

// void sendTelemetry();
// //void sendTelemetryPackage(uint8_t* flight, float* pitchAngle, float* altitudeMeter, float accX, float accY, float* accZ, float gyroX, float gyroY, float gyroZ, float* latitude, float* longitude, float* gpsAltitude);
// void calc_sensor();
// void getGPSInfo();
// void setup()
// {
//   Wire.begin();
//   Serial.begin(9600);
//   //ss.begin(115200);
// //   bmp.start(ULT_LOW_PWR_4p5ms);

//   // Read the WHO_AM_I register, this is a good test of communication
//   uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
//   Serial.print("I AM ");
//   Serial.print(c, HEX);
//   Serial.print(" I Should Be ");
//   Serial.println(0x68, HEX);

//   if (c == 0x68) // WHO_AM_I should always be 0x68
//   {
//     Serial.println("MPU6050 is online...");
//     if (mpu.MPU6050SelfTest()) {
//       Serial.println("Pass Selftest!");

//       mpu.setGscale(GFS_250DPS);
//       mpu.setAscale(AFS_2G);
//       mpu.calibrateMPU6050(); // Calibrate gyro and accelerometers, load biases in bias registers
//       mpu.initMPU6050(); //Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
//     }

//   else {
//       while (1)  // Loop forever if communication doesn't happen 
//       {
//         Serial.print("Could not connect to MPU6050: 0x");
//         Serial.println(c, HEX);
//       }
//     }
//   }
// }

// void loop()
// {
// //   static uint16_t now_millis = 0, last_time = 0; now_millis = millis();
//   calc_sensor();
//   sendTelemetry();
// //   Serial.print(now_millis - last_time); Serial.println(" >");
// //   last_time = now_millis;
// //   if(now_millis > 60000)
// //     now_millis = 0;
// //   if(last_time > 60000)
// //     last_time = 0;
// }
// void calc_sensor(){
//     // temperature = bmp.getTemperature();
//     // temperature *= 0.1;
//     // rakim = bmp.getAltitude();
//     if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) mpu.calculateAcc_Gyro();
//     Now = micros();
//     mpu.insertDeltat(((Now - lastUpdate) / 1000000.0f)); // set integration time by time elapsed since last filter update
//     lastUpdate = Now;

//     if(lastUpdate - 0L > 10000000uL) {
//         mpu.setBeta(0.041f); // decrease filter gain after stabilized
//         mpu.setZeta(0.015f); // increase gyro bias drift gain after stabilized
//     }

//     mpu.calculateEulerAngles(&yaw, &pitch, &roll); //yaw, pitch, roll angles are calculated in degrees.
//     SKF_az = skfAz.updateEstimateFloat(mpu.getAccZ());
//     SKF_pitch = skfPitch.updateEstimateFloat(pitch);
//     SKF_roll = skfRoll.updateEstimateFloat(roll);
//     SKF_yaw = skfYaw.updateEstimateFloat(yaw);
    
//     // while(ss.available() > 0)
//     //   if(gps.encode(ss.read()))
//     //     getGPSInfo();
// }

// // void getGPSInfo()
// // { 
// //   if (gps.location.isValid())
// //   {
// //     latitude = gps.location.lat();
// //     longitude = gps.location.lng();
// //   }
// //   else
// //   {
// //     latitude = 0.0000000f;
// //     longitude = 0.0000000f;
// //   }
// //   if(gps.altitude.isValid()){
// //     gpsAltitude = gps.altitude.meters();
// //   }
// //   else
// //     gpsAltitude = 0.00000f;
// // }



// void sendTelemetry() {
  
//   Serial.print(SKF_pitch, 2); Serial.print(","); Serial.print(SKF_roll, 2); Serial.print(",");
//   Serial.println(SKF_yaw,2);

// }

// // void sendTelemetryPackage(uint8_t* flight, float* pitchAngle, float* altitudeMeter, float accX, float accY, float* accZ, float gyroX, float gyroY, float gyroZ, float* latitude, float* longitude, float* gpsAltitude) {
// //   char dataPackage[69];
// //   dtostrf(*pitchAngle, 5, 2, dataPackage+2);
// //   dtostrf(*altitudeMeter, 7, 2, dataPackage+7);
// //   dtostrf(accX, 4, 2, dataPackage + 14);
// //   dtostrf(accY, 4, 2 ,dataPackage + 18);
// //   dtostrf(*accZ, 4, 2 , dataPackage + 22);
// //   dtostrf(gyroX, 6, 2, dataPackage + 26);
// //   dtostrf(gyroY, 6, 2, dataPackage + 32);
// //   dtostrf(gyroZ, 6, 2, dataPackage + 38);
// //   dtostrf(*latitude, 9, 6, dataPackage + 44);
// //   dtostrf(*longitude, 9, 6, dataPackage + 53);
// //   dtostrf(*gpsAltitude, 7, 2, dataPackage + 62); // 69

// //   dataPackage[0] = 31;
// //   dataPackage[1] = *flight + 30;
// //   for(uint8_t i_cnt = 0; i_cnt < 69; i_cnt++)
// //     Serial.print(dataPackage[i_cnt]);

// // }