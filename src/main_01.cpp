#include "Arduino.h"
#include "Wire.h"

#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

HMC5883L mag;
ITG3200 gyro;

int16_t mx, my, mz;
int16_t gx, gy, gz;

// class default I2C address is 0x53
// specific I2C addresses may be passed as a parameter here
// ALT low = 0x53 (default for SparkFun 6DOF board)
// ALT high = 0x1D
ADXL345 accel;

int16_t ax, ay, az;

#define LED_PIN 13 // (Arduino is 13, Teensy is 6)
bool blinkState = false;

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(14400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accel.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  mag.initialize();
  gyro.initialize();

  Serial.println("Testing device connections...");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
}

void loop()
{
  // read raw accel measurements from device
  accel.getAcceleration(&ax, &ay, &az);
  mag.getHeading(&mx, &my, &mz);
  gyro.getRotation(&gx, &gy, &gz);

  // Serial.print(az);
  //  display tab-separated accel x/y/z values
  Serial.print("accelga ");
  Serial.print(ax);
  Serial.print(" ");
  Serial.print(ay);
  Serial.print(" ");
  Serial.print(az);
  Serial.print(" ");

  Serial.print("magenta ");
  Serial.print(mx);
  Serial.print(" ");
  Serial.print(my);
  Serial.print(" ");
  Serial.print(mz);
  Serial.print(" ");

  Serial.print("gyroska ");
  Serial.print(gx);
  Serial.print(" ");
  Serial.print(gy);
  Serial.print(" ");
  Serial.print(az);
  Serial.print(" ");
  Serial.println();
  delay(10);

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}