// #include "SimpleKalmanFilter.h"
// #include "MPU6050.h"
// #include "SimpleKalmanFilter.h"

// MPU6050lib mpu;
// SimpleKalmanFilter gx = SimpleKalmanFilter(4.0f,4.0f,0.01f);
// SimpleKalmanFilter gy = SimpleKalmanFilter(4.0f, 4.0f, 0.01f);
// SimpleKalmanFilter gz = SimpleKalmanFilter(4.0f,4.0f,0.01f);

// float gyrX = 0, gyrY=0, gyrZ=0;
// static uint16_t now_millis = 0, last_time = 0;

// float pitch=0, yaw=0, roll=0; // FLIGHT EULER'S ANGLE
// void calc_sensor();
// void sendTelemetry();

// void setup()
// {
//   Wire.begin();
//   Serial.begin(9600);

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
//       mpu.initMPU6050();
//       Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope,
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
//   now_millis = micros();
//   float elapsedTime = 0.0f;
//   if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) mpu.calculateAcc_Gyro();   
//   gyrX = gx.updateEstimateFloat(mpu.getGyroX());
//   gyrY = gy.updateEstimateFloat(mpu.getGyroY());
//   gyrZ = gz.updateEstimateFloat(mpu.getGyroZ());
//   elapsedTime = (float) (now_millis - last_time) / 1000000.0f;
//   last_time = now_millis;
//   pitch += gyrX * elapsedTime;
//   roll += gyrY * elapsedTime;
//   yaw += gyrZ * elapsedTime;  

// }
// void sendTelemetry() {
//   Serial.print(pitch,2); Serial.print(","); Serial.print(roll,2); Serial.print(","); Serial.print(yaw,2); Serial.print(","); 
//   Serial.print(mpu.getGyroX(), 2); Serial.print(",");
//   Serial.print(mpu.getGyroY(), 2); Serial.print(","); 
//   Serial.println(mpu.getGyroZ(), 2);
// }
