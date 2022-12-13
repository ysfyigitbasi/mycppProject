// #include "bmp180.h"
// BMP180 bmp;
// long presPA;
// float temp, rakim;

// void setup(){
//     Serial.begin(9600);
//     Wire.begin();
//     bmp.start(ULT_LOW_PWR_4p5ms);
// }

// void loop(){
//     // static uint16_t now = 0, last_time = 0;
//     // now = millis();
//     temp = bmp.getTemperature();
//     temp *= 0.1;
//     // SKF_pressPA = SKF.updateEstimateLong(presPA);
//     rakim = bmp.getAltitude();

//     Serial.print("(Meter)\t"); //\t(SKF_Meter)
//     Serial.println(rakim);  //Serial.print("\t"); Serial.println(rakim);
//     // Serial.println(now - last_time);
//     // last_time = now;
//     // if(now > 60000)
//     //     now = 0;
//     // if(last_time > 60000)
//     //     last_time = 0;
// }