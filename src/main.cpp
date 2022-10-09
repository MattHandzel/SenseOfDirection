#include "Arduino.h"
#include "Wire.h"

#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

#include "HW579.h"

ADXL345 accel;

#define LED_PIN 6 // (Arduino is 13, Teensy is 6)
#define DATA_COLLECTION_BUTTON 3
bool blinkState = false;

// When getting orientation ask yourself:
// real life "{direction}" direction is chip "{direction}"
// so in this case, real life x is chip negative y
HW579 chip{{-2,-3,1}};
uint16_t startTime = 0;
void setup()
{
  Serial.begin(115200);
  while(!Serial);
  chip.Initialize(0);
  delay(6);
  startTime = millis();
}

int verbose = 1;
const bool printAccel = false;
const bool printGyro = true;
const bool printMag = false;
const int maxDataCount = 100;
int data[3][100];
int printCounter = 0;
int dataIndex = 0;


void integrateGyro(){
  static unsigned long endTime = millis();
  unsigned long startTime = millis();
  // actual_gx += (float) gx * (startTime - endTime) / 1000;
  // actual_gy += (float) gy * (startTime - endTime) / 1000;
  // actual_gz += (float) gz * (startTime - endTime) / 1000;
  endTime = millis();
}

uint64_t counter = 0;
void loop()
{

  chip.Update();
  // chip.PrintMag();
  if(++counter % 250 == 0){
    Serial.print(" gx: ");
    Serial.print(chip.gyroTheta, 2);
    Serial.print(" gy: ");
    Serial.print(chip.gyroPhi, 2);
    Serial.print(" gz: ");
    Serial.print(chip.gyroPsi, 2);
    Serial.print(" ax: ");
    Serial.print(chip.accelTheta, 2);
    Serial.print(" ay: ");
    Serial.print(chip.accelPhi, 2);
    Serial.print(" theta: ");
    Serial.print(chip.theta, 2);
    Serial.print(" phi: ");
    Serial.print(chip.phi, 2);
    Serial.print(" psi: ");
    Serial.print(chip.psi, 2);
    Serial.println();
    // chip.PrintGyro();
  }

  // float magData[3];
  // magData[0] = chip.magData[chip.orientation.x];
  // magData[1] = chip.magData[chip.orientation.y];
  // magData[2] = chip.magData[chip.orientation.z];
  // chip.PrintMag();
  // RotateVector3D(&magData[0], &magData[1], &magData[2], -chip.theta, -chip.phi, -chip.psi);
  // Serial.print("NEW MAG: x ");
  // Serial.print(magData[0]);
  // Serial.print(" y ");
  // Serial.print(magData[1]);
  // Serial.print(" z ");
  // Serial.println(magData[2]);



  // delay(1000 / 15);
}

// mag N -> -116; 497; -289
// mag S -> -136; -432; 185
// 220