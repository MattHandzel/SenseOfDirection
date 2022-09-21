#include <Arduino.h>
#include <stdio.h>
#include "MPU6050.h"
#include "Bracelet.h"

#define TAU PI * 2

/*Bracelet b = Bracelet();

MPU6050 mpu = MPU6050();

void setup()
{
  Serial.begin(115200);
  mpu.Initialize();
  mpu.StartCalibration(1000, 100);
  b.GenerateMotors();
}

void loop()
{
  mpu.GetReadings();
  mpu.CleanReadings();
  b.SetAngle(TAU - (mpu.GetRotation() / 180 * PI));
  mpu.ToSerial();
}*/
