
#include "Arduino.h"
#include "Wire.h"
#include "HMC5883L.h"

HMC5883L mag;
void setup(){
  Wire.begin();
  while(!Serial);
  mag.initialize();
  // Wire.beginTransmission(HMC5883L_ADDRESS);
  // Wire.write(0x00);
  // Wire.write(0x71);
  // Wire.endTransmission();
  // Wire.beginTransmission(HMC5883L_ADDRESS);
  // Wire.write(0x01);
  // Wire.write(0xA0);
  delay(6);
  mag.setGain(HMC5883L_GAIN_1370);
  mag.setMode(HMC5883L_MODE_CONTINUOUS);
  Serial.println(mag.testConnection());
}

int16_t mx, my, mz;
void loop(){
  
  mag.getHeading(&mx, &my, &mz);
  Serial.print(mx);
  Serial.print(" ");
  Serial.print(my);
  Serial.print(" ");
  Serial.println(mz);
  delay(100);
}