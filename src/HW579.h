#include "ADXL345.h"
#include "Arduino.h"
#include "HMC5883L.h"
#include "I2Cdev.h"
#include "ITG3200.h"
#include "Wire.h"

#define sign(x) (x / abs(x))

#define RADS_TO_DEGS 180 / PI

// ----- TODOS ----
// TODO: Check to see if higher sampling average for mag gives better results
// TODO: Check to see how sampling rate for gyro effects results
// TOOD: See how data output rate of magnetometer affects results
struct Orientation {
  uint8_t x;
  uint8_t y;
  uint8_t z;

  int xSign;
  int ySign;
  int zSign;
  Orientation(int _x, int _y, int _z) {
    xSign = sign(_x);
    ySign = sign(_y);
    zSign = sign(_z);

    x = abs(_x) - 1;
    y = abs(_y) - 1;
    z = abs(_z) - 1;
  }
};

void RotateVector3D(float *x, float *y, float *z, float theta, float phi,
                    float psi) {
  // orientation will be given in rads
  float xOld = *x;
  float yOld = *y;
  float zOld = *z;
  *x = xOld * cos(psi) - yOld * sin(psi);
  *y = xOld * sin(psi) + yOld * cos(psi);
  *z = zOld; // z doesn't change, we are rotating around that axis

  xOld = *x;
  yOld = *y;
  zOld = *z;
  *x = xOld * cos(phi) + zOld * sin(phi);
  *y = yOld;
  *z = -1 * xOld * sin(phi) + zOld * cos(phi);

  xOld = *x;
  yOld = *y;
  zOld = *z;
  *x = xOld;
  *y = yOld * cos(theta) - zOld * sin(theta);
  *z = yOld * sin(theta) + zOld * cos(theta);
}

class HW579 {
public:
  // Constructor, in here initialize the gyro, accel, and magnetometer
  HW579(Orientation _orientation) : orientation(_orientation) {}

  void Initialize(int verbose = 0) {
    if (verbose > 0) {
      Serial.println("Initializing HW579");
    }
    Wire.begin();

    accel.initialize();

    //? Why I am doing this
    // I think this is arbitrary, I need to run the acceleromater straight up
    // and then find off the actual offsets and put them here. This will allow
    // me to do more cool stuff!
    accel.setOffsetX(-12 / 4); // Offset by 12
    accel.setOffsetY(-12 / 4); // Offset by 12
    accel.setOffsetZ(12 / 4);  // Offset by 12
    switch (accel.getRange()) {
    case 0x0: // +- 2g resolution
      ACCEL_RAW_TO_G = 2 / 512.0;
      break;
    case 0x1:
      ACCEL_RAW_TO_G = 4 / 512.0;
      break;
    case 0x2:
      ACCEL_RAW_TO_G = 8 / 512.0;
      break;
    case 0x3:
      ACCEL_RAW_TO_G = 16 / 512.0;
      break;
    }

    // Change rate of magnetometer
    mag.initialize(); // intiailize magentormeter
    mag.setGain(HMC5883L_GAIN_1370);
    mag.setMode(HMC5883L_MODE_SINGLE);
    mag.setDataRate(HMC5883L_RATE_75);
    // mag.setSampleAveraging

    MAG_REFRESH_RATE = floor(1000 / 75) + 1; // 75 Hz

    MAG_RAW_TO_GUASS = (0.88 / 4096.0);

    // Gyro can update at anywhere from 1 kHz to 5 hz
    // https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf

    gyro.initialize(); // initialize gyroscope
    gyro.setRate(7);
    gyro.setDLPFBandwidth(ITG3200_DLPF_BW_5);
    GYRO_REFRESH_RATE = 1000 / (1 + gyro.getRate());
    GYRO_RAW_TO_DEG_PER_SEC = 2000 / 32768.0;
    if (verbose > 0) {
      Serial.println("Testing device connections...");
      Serial.println(mag.testConnection() ? "HMC5883L connection successful"
                                          : "HMC5883L connection failed");
      Serial.println(accel.testConnection() ? "ADXL345 connection successful"
                                            : "ADXL345 connection failed");
    }

    // Get some readings from the gyro sensor (for some reason the gyro likes to
    // have wierd values initially)
    int16_t gyroX = 0, gyroY = 0, gyroZ = 0;
    for (int i = 0; i < 1 * (1000 / GYRO_REFRESH_RATE); i++) {
      gyro.getRotation(&gyroX, &gyroY, &gyroZ);
      delay(GYRO_REFRESH_RATE);
    }
  }

  void Calibrate(int _totalCalibrationTime) {
    // Use interrput pins to get readings from the accel, gyro, and mag

    // Compute vector magnitude for accelerometer, and figure out coefficient to
    // make it equal to 9.807 m/s^2

    // Compute gyro drift rate here, make that as an offset

    // Calibrate magnetometer (move it arround so that we can get the proper
    // offset)

    // Write the calibration values to the gyro, accel, and magnetometer to
    // EEPROM (or flash memeory)
  }

  void FindNorth() {
    // TODO I can also find the minimum magnetic field and then set that to
    // -180, ykyk

    static float maxMagField = -99999999999999999;
    static float northOffset = 0;
    static bool rotatedOnce = false;

    if (magData[orientation.z] * magData[orientation.z] *
                sign(magData[orientation.z]) +
            magData[orientation.x] * magData[orientation.x] *
                sign(magData[orientation.x]) +
            magData[orientation.y] * magData[orientation.y] *
                sign(magData[orientation.y]) >
        maxMagField) {
      maxMagField = magData[orientation.z] * magData[orientation.z] *
                        sign(magData[orientation.z]) +
                    magData[orientation.x] * magData[orientation.x] *
                        sign(magData[orientation.x]) +
                    magData[orientation.y] * magData[orientation.y] *
                        sign(magData[orientation.y]);
      northOffset = gyroPsi;
    }

    if (gyroPsi > 360 || gyroPsi < -360) {
      rotatedOnce = true;
    }
    if (rotatedOnce) {
      angleOffOfNorth = fmod(-gyroPsi + northOffset - 180, 360);
    }
  }

  bool CalibrateMagnetometer(float _calibrationTime) {
    // Calibrate a magnetometer by computing the maximum magnetic field that it
    // senses for each axis and using that as an offset additionally, it uses
    // the maximum magnetic field sensed to scale each axis and normalize them
    // to one
    int16_t magX, magY, magZ;
    int16_t minX = 0, minY = 0, minZ = 0;
    int16_t maxX = 0, maxY = 0, maxZ = 0;
    int16_t xOff, yOff, zOff;
    int16_t xScale, yScale, zScale;

    int numSamples = _calibrationTime * (1000 / MAG_REFRESH_RATE);

    // Get the maximum and minimum values for each axis
    Serial.println("Calibrating magnetometer, move it around in a circle and "
                   "make sure there are no strong magnetic fields present");
    for (int i = 0; i < numSamples; i++) {
      mag.getHeading(&magX, &magY, &magZ);
      if (magX < minX)
        minX = magX;
      if (magX > maxX)
        maxX = magX;
      if (magY < minY)
        minY = magY;
      if (magY > maxY)
        maxY = magY;
      if (magZ < minZ)
        minZ = magZ;
      if (magZ > maxZ)
        maxZ = magZ;
      delay(MAG_REFRESH_RATE);
    }

    // Compute the offset and scale for each axis
    xOff = (maxX + minX) / 2;
    yOff = (maxY + minY) / 2;
    zOff = (maxZ + minZ) / 2;
    xScale = (maxX - minX) / 2;
    yScale = (maxY - minY) / 2;
    zScale = (maxZ - minZ) / 2;

    Serial.println("Magnetometer calibration complete");
    Serial.println("X Offset: " + String(xOff));
    Serial.println("Y Offset: " + String(yOff));
    Serial.println("Z Offset: " + String(zOff));
    Serial.println("X Scale: " + String(xScale));
    Serial.println("Y Scale: " + String(yScale));
    Serial.println("Z Scale: " + String(zScale));

    magXOffset = xOff;
    magYOffset = yOff;
    magZOffset = zOff;
    magXScale = xScale;
    magYScale = yScale;
    magZScale = zScale;
  }

  bool CalibrateGyro(float _calibrationTime = 10) {
    // Calibrate the gyro by taking the average of the readings over a period of
    // time This will be used to offset the gyro readings
    int16_t gyroX, gyroY, gyroZ;
    int numSamples =
        _calibrationTime *
        (1000 /
         GYRO_REFRESH_RATE); // How many seconds we will take the samples for
    Serial.println("Calibrating...");
    bool notDone = true;
    float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
    while (notDone) {
      float sumDeltaGyroX = 0, sumDeltaGyroY = 0, sumDeltaGyroZ = 0;
      int16_t gyroXPrev = 0, gyroYPrev = 0, gyroZPrev = 0;
      int32_t deltaGyroX = 0, deltaGyroY = 0, deltaGyroZ = 0;

      for (int i = 0; i < numSamples; i++) {

        // Get the derivative of gyro as a function of samples, if the
        // derivative changes quickly then cancel the calibration
        if (i > 0) {
          deltaGyroX = gyroXPrev - gyroX;
          deltaGyroY = gyroYPrev - gyroY;
          deltaGyroZ = gyroZPrev - gyroZ;
          sumDeltaGyroX += deltaGyroX;
          sumDeltaGyroY += deltaGyroY;
          sumDeltaGyroZ += deltaGyroZ;
          if (i > numSamples / 10) {
            if (deltaGyroX >= 2 * (sumDeltaGyroX / i + 2) ||
                deltaGyroX <= -2 * (sumDeltaGyroX / i + 2)) {
              Serial.println("Calibration failed, gyro moving too much around "
                             "x axis (theta)" +
                             String(deltaGyroX));
              return false;
            }
            if (deltaGyroY >= 2 * (sumDeltaGyroY / i + 2) ||
                deltaGyroY <= -2 * (sumDeltaGyroY / i + 2)) {
              Serial.println("Calibration failed, gyro moving too much around "
                             "y axis (phi)" +
                             String(deltaGyroY));
              return false;
            }
            if (deltaGyroZ >= 2 * (sumDeltaGyroZ / i + 2) ||
                deltaGyroZ <= -2 * (sumDeltaGyroZ / i + 2)) {
              Serial.println("Calibration failed, gyro moving too much around "
                             "z axis (psi)" +
                             String(deltaGyroZ));
              return false;
            }
          }
        }
        gyroXPrev = gyroX;
        gyroYPrev = gyroY;
        gyroZPrev = gyroZ;

        gyro.getRotation(&gyroX, &gyroY, &gyroZ);
        gyroXSum += gyroX;
        gyroYSum += gyroY;
        gyroZSum += gyroZ;

        if (i == numSamples - 1) {
          notDone = false;
          Serial.println("Calibration successful");
        }
        delay(GYRO_REFRESH_RATE);
      }
    }

    gyroXOffset = gyroXSum / numSamples;
    gyroYOffset = gyroYSum / numSamples;
    gyroZOffset = gyroZSum / numSamples;

    Serial.println("gyroXOffset =" + String(gyroXSum / numSamples));
    Serial.println("gyroYOffset =" + String(gyroYSum / numSamples));
    Serial.println("gyroZOffset =" + String(gyroZSum / numSamples));
    return true;
  }

  // This is the final function we want, we just want our current x, y, and z
  // rotation
  float GetRotation() {
    // rotation = sin(magData[0] / magData[1]);

    // rotation = atan(magData[abs(orientation.y)] /
    // magData[abs(orientation.x)]);

    // if (magData[orientation.x] < 0)
    // {
    //     theta += PI;
    // }

    // if (theta < 0)
    // {
    //     theta += 2 * PI;
    // }

    // Get the current roll and pitch based off of the accelerometer data

    return 0;

    Serial.print("accelRoll: ");
    Serial.print(accelTheta * RADS_TO_DEGS);
    Serial.print(" accelPitch: ");
    Serial.print(accelPhi * RADS_TO_DEGS);

    Serial.println();

    return 0;
    // float yaw = atan(accelData[orientation.z] /
    // sqrt(pow(accelData[orientation.x], 2) + pow(accelData[orientation.y],
    // 2))) * RADS_TO_DEGS;
    // Serial.println(String(atan(magData[abs(orientation.y)] /
    // magData[abs(orientation.x)]) * RADS_TO_DEGS) + "\t" + \
        //                String(atan(magData[abs(orientation.z)] /
    //                magData[abs(orientation.y)]) * RADS_TO_DEGS) + "\t");
    // Serial.println(String(atan(magData[abs(orientation.y)] /
    // magData[abs(orientation.x)]) * RADS_TO_DEGS) + "\t" + \
        //                String(atan(magData[abs(orientation.z)] /
    //                magData[abs(orientation.y)]) * RADS_TO_DEGS) + "\t");
  }

  void SyncronizeRotation() {
    // Get readings from accelerometer

    // Get readings from gyro

    // Get readings from magnetometer

    // Use the fact that the sum of the acceleration vector is equal to 9.8
    // (assuming that we aren't moving);=
  }

  void Update() {
    // Todo make gyro more accurate when going FAST
    currentTime = millis();
    float deltaTime = ((float)(currentTime - previousTime));
    // if (deltaTime <= updateIntervalMS)
    // {
    //     return;
    // }
    deltaTime /= 1e3;

    // Get the raw uint16_t values from the sensors
    accel.getAcceleration(&ax, &ay, &az);
    mag.getHeading(&mx, &my, &mz);
    gyro.getRotation(&gx, &gy, &gz);

    // Average filter for accelerometer, gyro, and mag (reduces noise)

    // offsets for gyro

    // gx += 25;
    // gy += -54;
    // gz += 9;

    // Convert data into actual numbers
    // Accel -
    // Gyro - Range is +- 2g |
    // Mag - Range is +- 8 guass = +-4096 | so one raw is equal to (8 / 4096.0)
    // guass

    magData[0] = (mx * MAG_RAW_TO_GUASS - magXOffset * MAG_RAW_TO_GUASS) /
                 (magXScale * MAG_RAW_TO_GUASS);
    magData[1] = (my * MAG_RAW_TO_GUASS - magYOffset * MAG_RAW_TO_GUASS) /
                 (magYScale * MAG_RAW_TO_GUASS);
    magData[2] = (mz * MAG_RAW_TO_GUASS - magZOffset * MAG_RAW_TO_GUASS) /
                 (magZScale * MAG_RAW_TO_GUASS);

    // adding the magnetic mapping (from calibration)
    // magData[orientation.x] = magData[orientation.x] * 1.009 +
    // magData[orientation.y] * -0.010 + magData[orientation.z] * 0.026;
    // magData[orientation.y] = magData[orientation.x] * -0.010 +
    // magData[orientation.y] * 1.025 + magData[orientation.z] * 0.114;
    // magData[orientation.z] = magData[orientation.x] * 0.026 +
    // magData[orientation.y] * 0.114 - magData[orientation.z] * 0.981;

    accelData[0] = ax * ACCEL_RAW_TO_G;
    accelData[1] = ay * ACCEL_RAW_TO_G;
    ;
    accelData[2] = az * ACCEL_RAW_TO_G;

    gyroData[0] =
        gx * GYRO_RAW_TO_DEG_PER_SEC - gyroXOffset * GYRO_RAW_TO_DEG_PER_SEC;
    gyroData[1] =
        gy * GYRO_RAW_TO_DEG_PER_SEC - gyroYOffset * GYRO_RAW_TO_DEG_PER_SEC;
    gyroData[2] =
        gz * GYRO_RAW_TO_DEG_PER_SEC - gyroZOffset * GYRO_RAW_TO_DEG_PER_SEC;

    // offsets

    // Swap axis around:
    magData[orientation.x] *= orientation.xSign;
    magData[orientation.y] *= orientation.ySign;
    magData[orientation.z] *= orientation.zSign;

    accelData[orientation.x] *= orientation.xSign;
    accelData[orientation.y] *= orientation.ySign;
    accelData[orientation.z] *= orientation.zSign;

    gyroData[orientation.x] *= orientation.xSign;
    gyroData[orientation.y] *= orientation.ySign;
    gyroData[orientation.z] *= orientation.zSign;

    // Median filter?

    // Print mag values before rotation vector

    // Rotate
    if (previousTime ==
        -1) // This makes it so that we ignore the first few attempts
    {
      previousTime = currentTime;
      return;
    }

    accelTheta =
        atan(accelData[orientation.x] / sqrt(pow(accelData[orientation.z], 2) +
                                             pow(accelData[orientation.y], 2)));

    // This makes is so that instead of the domain being [-90, 90], we increase
    // it to [-90, 270]
    if (accelData[orientation.z] < 0) {
      accelTheta = PI - accelTheta;
    }

    accelPhi =
        atan(accelData[orientation.y] / sqrt(pow(accelData[orientation.z], 2) +
                                             pow(accelData[orientation.x], 2)));
    // This makes is so that instead of the domain being [-90, 90], we increase
    // it to [-90, 270]
    if (accelData[orientation.z] < 0) {
      accelPhi = PI - accelPhi;
    }

    magPsi = atan((magData[orientation.y] / magData[orientation.x]));
    if (magData[orientation.y] > 0) {
      magPsi = PI - magPsi;
    }

    accelTheta *= RADS_TO_DEGS;
    accelPhi *= RADS_TO_DEGS;
    magPsi *= RADS_TO_DEGS;

    gyroTheta += gyroData[orientation.x] * deltaTime;
    gyroPhi += gyroData[orientation.y] * deltaTime;
    gyroPsi += gyroData[orientation.z] * deltaTime;

    float alpha = 0.5;

    theta = (theta + (gyroData[orientation.x] * deltaTime)) * (1 - alpha) +
            accelTheta * alpha;
    phi = (phi + (gyroData[orientation.y] * deltaTime)) * (1 - alpha) +
          accelPhi * alpha;
    psi = (psi + (gyroData[orientation.z] * deltaTime)) * (1 - alpha) +
          magPsi * alpha;

    previousTime = currentTime;
    // If this is the first time we have updated the sensor values then skip the
    // rest of this function
    if (previousUpdateTime == -1)
      return;
  }

  void ToString() {}

  void PrintData() {
    Serial.print("Accel: ");
    Serial.print(accelData[orientation.x]);
    Serial.print(", ");
    Serial.print(accelData[orientation.y]);
    Serial.print(", ");
    Serial.print(accelData[orientation.z]);
    Serial.print(" | ");

    Serial.print("Gyro: ");
    Serial.print(gyroData[orientation.x]);
    Serial.print(", ");
    Serial.print(gyroData[orientation.y]);
    Serial.print(", ");
    Serial.print(gyroData[orientation.z]);
    Serial.print(" | ");

    Serial.print("Mag: ");
    Serial.print(magData[orientation.x]);
    Serial.print(", ");
    Serial.print(magData[orientation.y]);
    Serial.print(", ");
    Serial.print(magData[orientation.z]);
    Serial.println();
  }

  void PrintMag() {
    Serial.print("Mag: ");
    Serial.print(magData[orientation.x]);
    Serial.print(" ");
    Serial.print(magData[orientation.y]);
    Serial.print(" ");
    Serial.println(magData[orientation.z]);
  }

  void PrintAccel() {
    Serial.print("Accel: ");
    Serial.print(accelData[orientation.x]);
    Serial.print(" ");
    Serial.print(accelData[orientation.y]);
    Serial.print(" ");
    Serial.print(accelData[orientation.z]);
    Serial.println();
  }

  void PrintGyro() {
    Serial.print("Gyro: ");
    Serial.print(gyroData[orientation.x]);
    Serial.print(", ");
    Serial.print(gyroData[orientation.y]);
    Serial.print(", ");
    Serial.print(gyroData[orientation.z]);
    Serial.println();
  }

  float gyroData[3];
  float accelData[3];
  float magData[3];

  HMC5883L mag;
  ITG3200 gyro;
  ADXL345 accel;

  float theta;
  float phi;
  float psi;

  float gyroTheta;
  float gyroPhi;
  float gyroPsi;

  float accelTheta;
  float accelPhi;

  float magTheta;
  float magPhi;
  float magPsi;

  u_int16_t currentTime = 0;
  u_int16_t previousTime = 0;

  Orientation orientation;

  float GYRO_REFRESH_RATE;

  float gyroXOffset = -24.53;
  float gyroYOffset = 53;
  float gyroZOffset = -8.06;

  float magXOffset = 0.11;
  float magYOffset = -35.88;
  float magZOffset = -72.29;

  float magXScale = 1;
  float magYScale = 1;
  float magZScale = 1;

  int16_t mx, my, mz;
  int16_t gx, gy, gz;
  int16_t ax, ay, az;

  float ACCEL_RAW_TO_G;
  float GYRO_RAW_TO_DEG_PER_SEC;
  float MAG_RAW_TO_GUASS;

  float MAG_REFRESH_RATE;

  int16_t currentRotation;
  int16_t previousRotation;

  float angleOffOfNorth = -123456;

protected:
  // class default I2C address is 0x53
  // specific I2C addresses may be passed as a parameter here
  // ALT low = 0x53 (default for SparkFun 6DOF board)
  // ALT high = 0x1D

  uint8_t verbose{0};

private:
  unsigned long previousUpdateTime = -1;
  bool isCalibrated = false;
  unsigned long updateIntervalMS = 10;
};
