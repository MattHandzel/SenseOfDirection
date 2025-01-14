#include "Arduino.h"
#include "Wire.h"

#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

#include "HW579.h"
#include "Bracelet.h"

// When getting orientation ask yourself:
// real life "{direction}" direction is chip "{direction}"
// so in this case, real life x is chip negative y
HW579 chip{{-2,-3,1}};

Orientation orientation = {1, 2, 3};

Bracelet bracelet{};

uint16_t startTime = 0;
void setup()
{
  pinMode(A6, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  bracelet.GenerateMotors();
  Serial.begin(115200);
  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);
  while(!Serial);
  chip.Initialize(0);
  delay(6);
  startTime = millis();
  
  // chip.CalibrateMagnetometer();
  while(!chip.CalibrateGyro(2));
}

int verbose = 1;
const bool printAccel = false;
const bool printGyro = true;
const bool printMag = false;
const int maxDataCount = 100;
int data[3][100];
int printCounter = 0;
int dataIndex = 0;

uint64_t counter = 0;

void loop()
{

  chip.Update();
  chip.FindNorth();

  // Randomass number to see if it has been updated yet
  if(chip.angleOffOfNorth == -123456){
    
  }
  else{
    Serial.println("Current heading is" + String(chip.angleOffOfNorth));
  }

  bracelet.SetAngle(chip.angleOffOfNorth * DEG_TO_RAD);
  bracelet.Update();

  Serial.println("\t");

  delay(1000 / 15);
}

// mag N -> -116; 497; -289
// mag S -> -136; -432; 185
// 220
