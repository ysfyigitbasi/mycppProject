#include "MPU6050.h"
#include "bmp180.h"
#include "SimpleKalmanFilter.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#define RELAY1 11
#define RELAY2 10

MPU6050lib mpu;
BMP180  bmp;
SoftwareSerial ss(8,7); // RX TX
TinyGPSPlus gps;

// Filtre Objeleri
SimpleKalmanFilter skfPitch = SimpleKalmanFilter(1.0f, 1.0f, 0.014f);
SimpleKalmanFilter skfAz = SimpleKalmanFilter(0.08f, 0.08f, 0.2f);
//uint32_t delt_t = 0;    // used to control display output rate
uint32_t lastUpdate = 0;// used to calculate integration interval
uint32_t Now_micros = 0;       // used to calculate integration interval

// // Euler Acilari
float pitch = 0.0f, yaw = 0.0f, roll = 0.0f; // FLIGHT EULER'S ANGLE

// Yukseklik ve sicaklik
float rakim = 0.0f, temperature = 0.0f, temp_alt = 0.0f, altitude = 0.0f;

// Filtrelenmis degiskenler
float SKF_pitch, SKF_az;

// GPS verileri
float latitude = 0.0f, longitude = 0.0f, gpsAltitude = 0.0f;

// Dikey Hiz
float verticalVelocity = 0.0f;
uint16_t timefor500ms = 0;
float dummy_temp_altitude = 0.0f;
bool vVelo = false;
bool apogee_altitude = false;
int vVeloCounter = 0;
float maximumAltitude = 0.0f;
int fallApogeeCounter = 0;

uint16_t now_millis = 0, last_time = 0, elapsedTime = 0;

bool count_fail_g = false, count_fail_alt = false;
bool count_start_g = false, count_start_alt = false;
bool kalkis_g = false, kalkis_alt = false;
uint8_t success_g = 0, success_alt = 0;


enum FLIGHT_STATE{
  ON_THE_RAMP = 0,
  FIRST_SEPERATION = 1,
  ONLY_SECOND_SEPERATION = 2,
  BOTH_SEPERATION = 3
};

int state = 0;
void getGPSInfo();
void sendTelemetry();
void calc_sensor();
void FireRelayR1();
void FireRelayR2();

void setup()
{
  Wire.begin();
  Serial.begin(115200);

  bmp.start(STANDART_7p5ms);

  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
  Serial.print("I AM ");
  Serial.print(c, HEX);
  Serial.print(" I Should Be ");
  Serial.println(0x68, HEX);

  if (c == 0x68) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU6050 is online...");
    if (mpu.MPU6050SelfTest()) {
      Serial.println("Pass Selftest!");

      mpu.setGscale(GFS_1000DPS);
      mpu.setAscale(AFS_8G);
      mpu.calibrateMPU6050(); // Calibrate gyro and accelerometers, load biases in bias registers
      mpu.initMPU6050(); Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    }
    else
    {
      Serial.print("Could not connect to MPU6050: 0x");
      Serial.println(c, HEX);
      while (1) ; // Loop forever if communication doesn't happen
    }
  }
  // kalman filtresi icin
  temperature = bmp.getTemperature();
  temperature *= 0.1;
  rakim = bmp.getAltitude();
  temperature = bmp.getTemperature();
  temperature *= 0.1;
  rakim = bmp.getAltitude();
  temperature = bmp.getTemperature();
  temperature *= 0.1;
  rakim = bmp.getAltitude();
  temperature = bmp.getTemperature();
  temperature *= 0.1;
  rakim = bmp.getAltitude();
  temperature = bmp.getTemperature();
  temperature *= 0.1;
  rakim = bmp.getAltitude();
  state = ON_THE_RAMP;
}

void loop()
{
  now_millis = millis(); // gerek yok sanirim
  temp_alt = altitude;
  calc_sensor();

  while( !(kalkis_g && kalkis_alt) ) {
    state = ON_THE_RAMP;
    temp_alt = altitude;
    calc_sensor();
    sendTelemetry();

    if(SKF_az > 3.0f) {kalkis_g = true;}
    if(altitude > 1000.0f) {kalkis_alt = true;}
    
    if(kalkis_alt && !kalkis_g){
        if (altitude > temp_alt) {
            if(!count_start_alt) {
                count_start_alt = true;
                success_alt = 1;
            }
            else {
                success_alt++;
            }
        }            
        if(count_fail_g) {
            success_g = 0;
            count_start_g = false;
            count_fail_g = false;
        }
        else {
            count_fail_g = true;
        }
    }
    if(!kalkis_alt && kalkis_g) {
        if(count_fail_alt) {
            success_alt = 0;
            count_start_alt = false;
            count_fail_alt = false;
        }
        else {
            count_fail_alt = true;
        }
        if(!count_start_g) {
            count_start_g = true;
            success_g = 1;
        }
        else {
            success_g++;
        }
    }
    if( (success_alt > 6) || (success_g > 6) ) {
        break;
    }

  }
  now_millis = 0; last_time = 0; elapsedTime = 0; timefor500ms = 0;
  now_millis = millis();
  temp_alt = altitude;
  dummy_temp_altitude = altitude;
  calc_sensor();
  sendTelemetry();

  elapsedTime = now_millis - last_time;
  last_time = now_millis;
  timefor500ms += elapsedTime;
  //verticalVelocity = (altitude - dummy_temp_altitude) / timefor500ms;
  vVelo = false;
  apogee_altitude = false;
  count_fail_alt = false;
  count_start_alt = false;
  success_alt = 0;

  while( !(vVelo || apogee_altitude) ) {
    state = ON_THE_RAMP;
    temp_alt = altitude;
    calc_sensor();
    sendTelemetry();
    elapsedTime = now_millis - last_time;
    last_time = now_millis;
    timefor500ms += elapsedTime;
    if (timefor500ms >= 500) {
      verticalVelocity = (altitude - dummy_temp_altitude) / timefor500ms;
      dummy_temp_altitude = altitude;
      if (altitude > 1500.0f) {
        if(verticalVelocity < 10.0f) { vVeloCounter++;} }
    }
    if(vVeloCounter > 2) {state  = FIRST_SEPERATION; vVelo = true; FireRelayR1(); }
    if (maximumAltitude > altitude)
    {
      if(!count_start_alt)
      {
        count_start_alt = true;
        success_alt = 1;
      }
      else{
        success_alt++;
      }
    }
    else{
      if (!count_fail_alt) {
        count_fail_alt = true;
      }
      else{
        count_fail_alt = false;
        success_alt = 0;
      }
    }
    if(success_alt > 4)
    {
      apogee_altitude = true;
      state  = FIRST_SEPERATION;
      FireRelayR1();
    }
  }
  state = FIRST_SEPERATION;
  temp_alt = altitude;
  calc_sensor();
  sendTelemetry();
  while (altitude > 600.0f){
    state = FIRST_SEPERATION;
    calc_sensor();
    sendTelemetry();
  }
  FireRelayR2();
  state = BOTH_SEPERATION;
  while (true)
  {
    calc_sensor();
    sendTelemetry();
  }

}

void FireRelayR1()
{
  pinMode(RELAY1 , OUTPUT);
  digitalWrite(RELAY1, HIGH);
  delay(500);
  digitalWrite(RELAY1, LOW);
  delay(50);

  digitalWrite(RELAY1, HIGH);
  delay(500);
  digitalWrite(RELAY1, LOW);
  delay(50);

  digitalWrite(RELAY1, HIGH);
  delay(500);
  digitalWrite(RELAY1, LOW);
}
void FireRelayR2()
{
  pinMode(RELAY2, OUTPUT);
  digitalWrite(RELAY2, HIGH);
  delay(500);
  digitalWrite(RELAY2, LOW);
  delay(50);

  digitalWrite(RELAY2, HIGH);
  delay(500);
  digitalWrite(RELAY2, LOW);
  delay(50);

  digitalWrite(RELAY2, HIGH);
  delay(500);
  digitalWrite(RELAY2, LOW);
}

void calc_sensor(){
    temperature = bmp.getTemperature();
    temperature *= 0.1;
    altitude = bmp.getAltitude() - rakim;
    if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) mpu.calculateAcc_Gyro();
    SKF_az = skfAz.updateEstimateFloat(mpu.getAccZ());
    Now_micros = micros();
    mpu.insertDeltat(((Now_micros - lastUpdate) / 1000000.0f)); // set integration time by time elapsed since last filter update
    lastUpdate = Now_micros;

    if(lastUpdate - 0L > 10000000uL) {
        mpu.setBeta(0.041f); // decrease filter gain after stabilized
        mpu.setZeta(0.015f); // increase gyro bias drift gain after stabilized
    }

    mpu.calculateEulerAngles(&yaw, &pitch, &roll); //yaw, pitch, roll angles are calculated in degrees.

    SKF_pitch = skfPitch.updateEstimateFloat(pitch);
    while(ss.available() > 0)
      if(gps.encode(ss.read()))
        getGPSInfo();
}
void getGPSInfo()
{ 
  if (gps.location.isValid())
  {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }
  else
  {
    latitude = 0.0000000f;
    longitude = 0.0000000f;
  }
  if(gps.altitude.isValid()){
    gpsAltitude = gps.altitude.meters();
  }
  else
    gpsAltitude = 0.00000f;
}

void sendTelemetry() {
  Serial.print("Orion,A,");Serial.print(altitude,2); Serial.print(","); Serial.print(gpsAltitude,2); Serial.print(",");
  Serial.print(latitude, 6); Serial.print(","); Serial.print(longitude,6); Serial.print(",");
  Serial.print(mpu.getGyroX(),2); Serial.print(","); Serial.print(mpu.getGyroY(),2); Serial.print(",");
  Serial.print(mpu.getGyroZ(),2); Serial.print(","); Serial.print(mpu.getAccX(),2); Serial.print(",");
  Serial.print(mpu.getAccY(),2); Serial.print(","); Serial.print(SKF_az,2); Serial.print(",");
  Serial.print(SKF_pitch,2); Serial.print(","); Serial.print(state); Serial.print(","); Serial.println("0.0,Orion");
}