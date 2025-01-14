
#include <Adafruit_SensorLab.h>
#include <Adafruit_Sensor_Calibration.h>

#include "Arduino.h"
#include "Wire.h"

#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"


#include "HW579.h"

// When getting orientation ask yourself:
// real life "{direction}" direction is chip "{direction}"
// so in this case, real life x is chip negative y
HW579 chip{{-2,-3,1}};

int verbose = 1;
const bool printAccel = false;
const bool printGyro = true;
const bool printMag = false;
const int maxDataCount = 100;
int data[3][100];
int printCounter = 0;
int dataIndex = 0;

uint64_t counter = 0;


int loopcount = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  chip.Initialize(0);
  delay(6);
  // chip.CalibrateMagnetometer();
  // while(!chip.CalibrateGyro());

}
void loop()
{

  chip.Update();
  Serial.print("Raw:");
  Serial.print(int(chip.accelData[chip.orientation.x] / chip.ACCEL_RAW_TO_G)); Serial.print(",");
  Serial.print(int(chip.accelData[chip.orientation.y] / chip.ACCEL_RAW_TO_G)); Serial.print(",");
  Serial.print(int(chip.accelData[chip.orientation.z] / chip.ACCEL_RAW_TO_G)); Serial.print(",");
  Serial.print(int(chip.gyroData[chip.orientation.x] / chip.GYRO_RAW_TO_DEG_PER_SEC)); Serial.print(",");
  Serial.print(int(chip.gyroData[chip.orientation.y] / chip.GYRO_RAW_TO_DEG_PER_SEC)); Serial.print(",");
  Serial.print(int(chip.gyroData[chip.orientation.z] / chip.GYRO_RAW_TO_DEG_PER_SEC)); Serial.print(",");
  Serial.print(int(chip.magData[chip.orientation.x])); Serial.print(",");
  Serial.print(int(chip.magData[chip.orientation.y])); Serial.print(",");
  Serial.print(int(chip.magData[chip.orientation.z])); Serial.println("");

  // unified data
  Serial.print("Uni:");
  Serial.print(int(chip.accelData[chip.orientation.x])); Serial.print(",");
  Serial.print(int(chip.accelData[chip.orientation.y])); Serial.print(",");
  Serial.print(int(chip.accelData[chip.orientation.z])); Serial.print(",");
  Serial.print(int(chip.gyroData[chip.orientation.x])); Serial.print(",");
  Serial.print(int(chip.gyroData[chip.orientation.y])); Serial.print(",");
  Serial.print(int(chip.gyroData[chip.orientation.z])); Serial.print(",");
  Serial.print(int(chip.magData[chip.orientation.x])); Serial.print(",");
  Serial.print(int(chip.magData[chip.orientation.y])); Serial.print(",");
  Serial.print(int(chip.magData[chip.orientation.z])); Serial.println("");
  delay(1000 / 15);
}
