#include "Arduino.h"
#include "Wire.h"

#include "ADXL345.h"
#include "HMC5883L.h"
#include "I2Cdev.h"
#include "ITG3200.h"

#include "Bracelet.h"
#include "HW579.h"
// When getting orientation ask yourself:
// real life "{direction}" direction is chip "{direction}"
// so in this case, real life x is chip negative y
HW579 chip{{-2, -3, 1}};

Orientation orientation = {1, 2, 3};

Bracelet bracelet{};

uint16_t startTime = 0;
void setup() {
  pinMode(A6, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  bracelet.GenerateMotors();
  Serial.begin(115200);
  while (!Serial)
    ;
  // chip.Initialize(0);
  delay(100);
  Serial.println("asd");
  startTime = millis();

  Serial.println("Performing test...");
  Serial.println(chip.accel.testConnection());
  Serial.println(chip.gyro.testConnection());
  Serial.println(chip.mag.testConnection());

  Serial.println("hi");
  // chip.CalibrateMagnetometer();
  // while(!chip.CalibrateGyro(2));
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

void loop() {
  // chip.Update();
  // chip.FindNorth();
  // chip.PrintAccel();
  // chip.PrintGyro();
  // chip.PrintMag();
  // Randomass number to see if it has been updated yet
  if (chip.angleOffOfNorth == -123456) {
    bracelet.SetStatus(CALIBRATING);
  } else {
    bracelet.SetStatus(RUNNING);
    Serial.println("Current heading is" + String(chip.angleOffOfNorth));
    bracelet.SetAngle(chip.angleOffOfNorth * DEG_TO_RAD);
  }

  // analogWrite(A7, 255);
  // analogWrite(A1, 255);
  // analogWrite(A8, 255);
  // analogWrite(A6, 255);
  // analogWrite(A6, 255);

  // bracelet.Update();

  Serial.println("\t");

  delay(1000 / 15);
}

// mag N -> -116; 497; -289
// mag S -> -136; -432; 185
// 220
