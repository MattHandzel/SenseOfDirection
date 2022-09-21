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

#define LED_PIN 6 // (Arduino is 13, Teensy is 6)
#define DATA_COLLECTION_BUTTON 3
bool blinkState = false;

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  pinMode(LED_PIN, OUTPUT);
  pinMode(DATA_COLLECTION_BUTTON, INPUT_PULLUP);

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(115200);

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
  delay(6);
}

int verbose = 0;
const bool printAccel = false;
const bool printGyro = false;
const bool printMag = true;
const int maxDataCount = 100;
int data[3][100];
int printCounter = 0;
int dataIndex = 0;
void loop()
{
  // read raw accel measurements from device
  accel.getAcceleration(&ax, &ay, &az);
  mag.getHeading(&mx, &my, &mz);
  gyro.getRotation(&gx, &gy, &gz);


  // Experimental data shows that mz is bigger than expected
  mz *= 0.63;
  mz += 50; // Experimental offset


  my += 37; // experimental offset

  if(my < -270){
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("North");
  }

  // Serial.print(az);
  //  display tab-separated accel x/y/z values
  if (verbose >= 1)
  {
    if (printAccel)
    {
      Serial.print("accel ");
      Serial.print(ax);
      Serial.print(" ");
      Serial.print(ay);
      Serial.print(" ");
      Serial.print(az);
      Serial.print(" ");
    }
    if (printMag)
    {
      Serial.print("mag ");
      Serial.print(mx);
      Serial.print(" ");
      Serial.print(my);
      Serial.print(" ");
      Serial.println(mz);
    }
    if (printGyro)
    {
      // Serial.print("gyro ");
      Serial.print(gx);
      Serial.print(" ");
      Serial.print(gy);
      Serial.print(" ");
      Serial.print(az);
      Serial.print(" ");
      Serial.println();
    }
  }

  if (Serial && printCounter < dataIndex) //(Serial)
  {
    printCounter++;
    printCounter = printCounter > dataIndex ? dataIndex : printCounter;
    Serial.print(data[0][printCounter]);
    Serial.print(" ");
    Serial.print(data[1][printCounter]);
    Serial.print(" ");
    Serial.println(data[2][printCounter]);
  }
  if (!digitalRead(DATA_COLLECTION_BUTTON))
  {
    if (dataIndex < maxDataCount)
    {
      data[0][dataIndex] = mx;
      data[1][dataIndex] = my;
      data[2][dataIndex] = mz;
    }
    else
    {
      digitalWrite(LED_PIN, HIGH);
    }
  }

  delay(1000 / 15);
}

// mag N -> -116; 497; -289
// mag S -> -136; -432; 185
// 220