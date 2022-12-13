#include "MPU6050.h"
// Set initial input parameters

float MPU6050lib::getGres() const{
  return gRes;
}
void MPU6050lib::calc_Gres(){
    switch (myGscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes= 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes= 2000.0 / 32768.0;
      break;
    default: //error message
      Serial.println("ERROR: UNKNOWN GYROSCALE!");
  }
}
void MPU6050lib::calc_Ares(){
    switch (myAscale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes= 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes= 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes= 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes= 16.0 / 32768.0;
      break;
    default: //error message
      Serial.println("ERROR: UNKNOWN ACCELOMETERSCALE!");
  }
}
float MPU6050lib::getAres() const{
  return aRes;
}
uint8_t MPU6050lib::getGscale() const{
  return myGscale;
}
uint8_t MPU6050lib::getAscale() const{
  return myAscale;
}
void MPU6050lib::setGscale(int Gyroscale) {
	myGscale = Gyroscale;
  calc_Gres();
}
void MPU6050lib::setAscale(int Accscale) {
	myAscale = Accscale;
  calc_Ares();
}
void MPU6050lib::readAccelData()
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  accelCount[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  accelCount[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;
  accelCount[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ;
}

void MPU6050lib::readGyroData()
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gyroCount[0] = (int16_t)((rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gyroCount[1] = (int16_t)((rawData[2] << 8) | rawData[3]) ;
  gyroCount[2] = (int16_t)((rawData[4] << 8) | rawData[5]) ;
}

void MPU6050lib::calculateAcc_Gyro(){
  readAccelData();
  readGyroData();
  // Now we'll calculate the accleration value into actual g's
  ax = (float)accelCount[0] * aRes; // get actual g value, this depends on scale being set
  ay = (float)accelCount[1] * aRes;
  az = (float)accelCount[2] * aRes;
  gyrox = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
  gyroy = (float)gyroCount[1] * gRes;
  gyroz = (float)gyroCount[2] * gRes;
}

float MPU6050lib::getAccX() const{
  return ax;
}
float MPU6050lib::getAccY() const{
  return ay;
}
float MPU6050lib::getAccZ() const{
  return az;
}
float MPU6050lib::getGyroX() const{
  return gyrox;  
}
float MPU6050lib::getGyroY() const{
  return gyroy;  
}
float MPU6050lib::getGyroZ() const{
  return gyroz;  
}
void MPU6050lib::insertDeltat(const float timesInMicros){
  deltat = timesInMicros;
}
void MPU6050lib::setBeta(const float B){
  beta = B;
}
void MPU6050lib::setZeta(const float Z){
  zeta = Z;
}
// bool MPU6050lib::directionCosines(float* Aroll, float* Apitch, float* Ayaw) {
//   // float C_mat [3][3] = {{0,G_z,-G_x}, {-G_z,0,G_y}, {G_x,-G_y,0}};
//   static float Tnew [3][3] = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f}};

//   Tnew[0][0] = T[1][0]*gyrox*PI/180.0f - gyroy*T[2][0]*PI/180.0f;
//   Tnew[0][1] = T[1][1]*gyrox*PI/180.0f - gyroy*T[2][1]*PI/180.0f;
//   Tnew[0][2] = gyrox*T[1][2]*PI/180.0f - gyroy*T[2][2]*PI/180.0f;
  
//   Tnew[1][0] = -gyrox*T[0][0]*PI/180.0f + gyroz*T[2][0]*PI/180.0f;
//   Tnew[1][1] = -gyrox*T[0][1]*PI/180.0f + gyroz*T[2][1]*PI/180.0f;
//   Tnew[1][2] = -gyrox*T[0][2]*PI/180.0f + gyroz*T[2][2]*PI/180.0f;

//   Tnew[2][0] = gyroy*T[0][0]*PI/180.0f - gyroz*T[1][0]*PI/180.0f;
//   Tnew[2][1] = gyroy*T[0][1]*PI/180.0f - gyroz*T[1][1]*PI/180.0f;
//   Tnew[2][2] = gyroy*T[0][2]*PI/180.0f - gyroz*T[1][2]*PI/180.0f;

//   *Apitch = asinf(-Tnew[0][2]) * 180.0f / PI;

//   if(*Aroll > 0.0f) {
//     if(*Aroll > 85.0f && (*Aroll < 95.0f) ) {
//       T[0][0] = Tnew[0][0]; T[0][1] = Tnew[0][1]; T[0][2] = Tnew[0][2];
//       T[1][0] = Tnew[1][0]; T[1][1] = Tnew[1][1]; T[1][2] = Tnew[1][2];
//       T[2][0] = Tnew[2][0]; T[2][1] = Tnew[2][1]; T[2][2] = Tnew[2][2];
//       return true;
//     }
//     if(*Aroll > 265.0f && (*Aroll < 275.0f)) {
//       T[0][0] = Tnew[0][0]; T[0][1] = Tnew[0][1]; T[0][2] = Tnew[0][2];
//       T[1][0] = Tnew[1][0]; T[1][1] = Tnew[1][1]; T[1][2] = Tnew[1][2];
//       T[2][0] = Tnew[2][0]; T[2][1] = Tnew[2][1]; T[2][2] = Tnew[2][2];
//       return true;  
//     }
//   }
//   else{
//     if(*Aroll < -85.0f && (*Aroll > -95.0f) ) {
//       T[0][0] = Tnew[0][0]; T[0][1] = Tnew[0][1]; T[0][2] = Tnew[0][2];
//       T[1][0] = Tnew[1][0]; T[1][1] = Tnew[1][1]; T[1][2] = Tnew[1][2];
//       T[2][0] = Tnew[2][0]; T[2][1] = Tnew[2][1]; T[2][2] = Tnew[2][2];
//       return true;
//     }
//     if(*Aroll < -265.0f && (*Aroll > -275.0f)) {
//       T[0][0] = Tnew[0][0]; T[0][1] = Tnew[0][1]; T[0][2] = Tnew[0][2];
//       T[1][0] = Tnew[1][0]; T[1][1] = Tnew[1][1]; T[1][2] = Tnew[1][2];
//       T[2][0] = Tnew[2][0]; T[2][1] = Tnew[2][1]; T[2][2] = Tnew[2][2];
//       return true;  
//     }
//   }

//   // if(Tnew[0][1] < 0) {
//   //   *Apitch = -acosf(Tnew[0][0] / cos((*Aroll) * PI / 180.0f)); }
//   // else{
//   *Ayaw = atan2f(Tnew[0][1] , Tnew[0][0]) * 180.0f / PI;
    
//   // if(Tnew[1][2] < 0) {
//   //   *Ayaw = -acosf(Tnew[2][2] / cos((*Aroll) * PI / 180.0f)); }
//   // else{
//   *Aroll = atan2f(Tnew[1][2] , Tnew[2][2]) * 180.0f / PI;
//   T[0][0] = Tnew[0][0]; T[0][1] = Tnew[0][1]; T[0][2] = Tnew[0][2];
//   T[1][0] = Tnew[1][0]; T[1][1] = Tnew[1][1]; T[1][2] = Tnew[1][2];
//   T[2][0] = Tnew[2][0]; T[2][1] = Tnew[2][1]; T[2][2] = Tnew[2][2];
//   return false;
// }

void MPU6050lib::MadgwickQuaternionUpdate(float Acc_x, float Acc_y, float Acc_z, float Gyro_x, float Gyro_y, float Gyro_z){
// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!

  float q1 = vector[0], q2 = vector[1], q3 = vector[2], q4 = vector[3];         // short name local variable for readability
  float norm;                                               // vector norm
  float f1, f2, f3;                                         // objetive funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error
    // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(Acc_x * Acc_x + Acc_y * Acc_y + Acc_z * Acc_z);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  Acc_x *= norm;
  Acc_y *= norm;
  Acc_z *= norm;
            
  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - Acc_x;
  f2 = _2q1 * q2 + _2q3 * q4 - Acc_y;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - Acc_z;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;
          
  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;
            
  // Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;
            
  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
            
//  Compute and remove gyroscope biases
  gbiasx += gerrx * deltat * zeta;
  gbiasy += gerry * deltat * zeta;
  gbiasz += gerrz * deltat * zeta;
  Gyro_x -= gbiasx;
  Gyro_y -= gbiasy;
  Gyro_z -= gbiasz;
            
// Compute the quaternion derivative
  qDot1 = -_halfq2 * Gyro_x - _halfq3 * Gyro_y - _halfq4 * Gyro_z;
  qDot2 =  _halfq1 * Gyro_x + _halfq3 * Gyro_z - _halfq4 * Gyro_y;
  qDot3 =  _halfq1 * Gyro_y - _halfq2 * Gyro_z + _halfq4 * Gyro_x;
  qDot4 =  _halfq1 * Gyro_z + _halfq2 * Gyro_y - _halfq3 * Gyro_x;

// Compute then integrate estimated quaternion derivative
  q1 += (qDot1 -(beta * hatDot1)) * deltat;
  q2 += (qDot2 -(beta * hatDot2)) * deltat;
  q3 += (qDot3 -(beta * hatDot3)) * deltat;
  q4 += (qDot4 -(beta * hatDot4)) * deltat;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  vector[0] = q1 * norm;
  vector[1] = q2 * norm;
  vector[2] = q3 * norm;
  vector[3] = q4 * norm;
}
void MPU6050lib::calculateEulerAngles(float* yaw, float* pitch, float* roll){
    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    MadgwickQuaternionUpdate(getAccX(), getAccY(), getAccZ(), getGyroX() * PI / 180.0f,
    getGyroY() * PI / 180.0f, getGyroZ() * PI / 180.0f);
    *yaw    = atan2(2.0f * (vector[1] * vector[2] + vector[0] * vector[3]), vector[0] * vector[0] + vector[1] * vector[1] - vector[2] * vector[2] - vector[3] * vector[3]);
    *pitch  = -asin(2.0f * (vector[1] * vector[3] - vector[0] * vector[2]));
    *roll   = atan2(2.0f * (vector[0] * vector[1] + vector[2] * vector[3]), vector[0] * vector[0] - vector[1] * vector[1] - vector[2] * vector[2] + vector[3] * vector[3]);

    *yaw *= 180.0f / PI;
    *pitch *= 180.0f / PI;
    *roll *= 180.0f / PI;
}
// // void MPU6050lib::getEulerAngels(float* x, float* y, float* z) const{
// //   *x = yaw;
// //   *y = pitch;
// //   *z = roll;
// // }
// // Configure the motion detection control for low power accelerometer mode
// void MPU6050lib::LowPowerAccelOnlyMPU6050()
// {

//   // The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
//   // Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
//   // above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a
//   // threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
//   // consideration for these threshold evaluations; otherwise, the flags would be set all the time!

//   uint8_t c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
//   writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x30); // Clear sleep and cycle bits [5:6]
//   writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x30); // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

//   c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
//   writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0x38); // Clear standby XA, YA, and ZA bits [3:5]
//   writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x00); // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running

//   c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
//   writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
//   // Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
//   writeByte(MPU6050_ADDRESS, ACCEL_CONFIG,  c | 0x00);  // Set ACCEL_HPF to 0; reset mode disbaling high-pass filter

//   c = readByte(MPU6050_ADDRESS, CONFIG);
//   writeByte(MPU6050_ADDRESS, CONFIG, c & ~0x07); // Clear low-pass filter bits [2:0]
//   writeByte(MPU6050_ADDRESS, CONFIG, c |  0x00);  // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate

//   c = readByte(MPU6050_ADDRESS, INT_ENABLE);
//   writeByte(MPU6050_ADDRESS, INT_ENABLE, c & ~0xFF);  // Clear all interrupts
//   writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x40);  // Enable motion threshold (bits 5) interrupt only

//   // Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
//   // for at least the counter duration
//   writeByte(MPU6050_ADDRESS, MOT_THR, 0x80); // Set motion detection to 0.256 g; LSB = 2 mg
//   writeByte(MPU6050_ADDRESS, MOT_DUR, 0x01); // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate

//   delay (100);  // Add delay for accumulation of samples

//   c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
//   writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
//   writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c |  0x07);  // Set ACCEL_HPF to 7; hold the initial accleration value as a referance

//   c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
//   writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0xC7); // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
//   writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x47); // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])

//   c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
//   writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x20); // Clear sleep and cycle bit 5
//   writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x20); // Set cycle bit 5 to begin low power accelerometer motion interrupts

// }


void MPU6050lib::initMPU6050()
{
  // wake up device-don't need this here if using calibration function below
  //  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  //  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

  // get stable time source
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

  // Configure Gyro and Accelerometer
  // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
  // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
  // Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
  writeByte(MPU6050_ADDRESS, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | getGscale() << 3); // Set full scale range for the gyro

  // Set accelerometer configuration
  c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | getAscale() << 3); // Set full scale range for the accelerometer

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU6050lib::calibrateMPU6050()
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device, reset all registers, clear gyro and accelerometer bias registers
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source
  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00 | getGscale() << 3); // Set full scale range for the gyro
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00 | getAscale() << 3); // Set full scale range for the gyro

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
  delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) 1/getAres(); // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) 1/getAres();
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

  gyroBias[0] = (float) gyro_bias[0] * getGres(); // construct gyro bias in deg/s for later manual subtraction
  gyroBias[1] = (float) gyro_bias[1] * getGres();
  gyroBias[2] = (float) gyro_bias[2] * getGres();

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Push accelerometer biases to hardware registers
  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]); // might not be supported in MPU6050
  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

  // Output scaled accelerometer biases for manual subtraction in the main program
  accelBias[0] = (float)accel_bias[0] * getAres();
  accelBias[1] = (float)accel_bias[1] * getAres();
  accelBias[2] = (float)accel_bias[2] * getAres();

  Serial.println("MPU6050 bias");
  Serial.println(" x\t  y\t  z  ");
  Serial.print((int)(1000 * accelBias[0])); Serial.print('\t');
  Serial.print((int)(1000 * accelBias[1])); Serial.print('\t');
  Serial.print((int)(1000 * accelBias[2]));
  Serial.println(" mg");

  Serial.print(gyroBias[0], 1); Serial.print('\t');
  Serial.print(gyroBias[1], 1); Serial.print('\t');
  Serial.print(gyroBias[2], 1);
  Serial.println(" o/s");
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
bool MPU6050lib::MPU6050SelfTest() // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[4];
  uint8_t selfTest[6];
  float factoryTrim[6];

  // Configure the accelerometer for self-test
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(250);  // Delay a while to let the device execute the self-test
  rawData[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X); // X-axis self-test results
  rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
  rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
  rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
  // Extract the acceleration test results first
  selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
  selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
  selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) ; // ZA_TEST result is a five-bit unsigned integer
  // Extract the gyration test results first
  selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
  selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
  selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
  // Process results to allow final comparison with factory set values
  factoryTrim[0] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[0] - 1.0) / 30.0))); // FT[Xa] factory trim calculation
  factoryTrim[1] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[1] - 1.0) / 30.0))); // FT[Ya] factory trim calculation
  factoryTrim[2] = (4096.0 * 0.34) * (pow( (0.92 / 0.34) , (((float)selfTest[2] - 1.0) / 30.0))); // FT[Za] factory trim calculation
  factoryTrim[3] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[3] - 1.0) ));         // FT[Xg] factory trim calculation
  factoryTrim[4] =  (-25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[4] - 1.0) ));         // FT[Yg] factory trim calculation
  factoryTrim[5] =  ( 25.0 * 131.0) * (pow( 1.046 , ((float)selfTest[5] - 1.0) ));         // FT[Zg] factory trim calculation

  //  Output self-test results and factory trim calculation if desired
  //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
  //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
  //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
  //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get to percent, must multiply by 100 and subtract result from 100
  for (int i = 0; i < 6; i++) {
    SelfTest[i] = 100.0 + 100.0 * ((float)selfTest[i] - factoryTrim[i]) / factoryTrim[i]; // Report percent differences
  }
// Start by performing self test and reporting values
//    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
//    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
//    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
//    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
//    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
//    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
  return (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f);
}

void MPU6050lib::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t MPU6050lib::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);	                 // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void MPU6050lib::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}
