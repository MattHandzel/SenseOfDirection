#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

#define sign(x) (x / abs(x))

#define RADS_TO_DEGS 180 / PI

struct Orientation
{
    uint8_t x;
    uint8_t y;
    uint8_t z;

    int xSign;
    int ySign;
    int zSign;
    Orientation(int _x, int _y, int _z){
        xSign = sign(_x);
        ySign = sign(_y);
        zSign = sign(_z);

        x = abs(_x)-1;
        y = abs(_y)-1;
        z = abs(_z)-1;
    }
};

void RotateVector3D(float *x, float *y, float*z, float theta, float phi, float psi){
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


class HW579
{
public:
    // Constructor, in here initialize the gyro, accel, and magnetometer
    HW579(Orientation _orientation) : orientation(_orientation)
    {
    }

    void Initialize(int verbose = 0)
    {
        if (verbose > 0)
        {
            Serial.println("Initializing HW579");
        }
        Wire.begin();
        accel.initialize();
        accel.setOffsetX(-12/4); // Offset by 12
        accel.setOffsetY(-12/4); // Offset by 12
        accel.setOffsetZ(12/4); // Offset by 12
        switch (accel.getRange())
        {
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
        mag.initialize();  // intiailize magentormeter
        mag.setGain(HMC5883L_GAIN_1370);
        mag.setMode(HMC5883L_MODE_SINGLE);
        // mag.setSampleAveraging
        MAG_RAW_TO_GUASS = (0.88 / 2048.0);

        gyro.initialize(); // initialize gyroscope
        gyro.setRate(7);
        gyro.setDLPFBandwidth(ITG3200_DLPF_BW_5);
        GYRO_RAW_TO_DEG_PER_SEC = 2000 / 32768.0;
        if (verbose > 0)
        {
            Serial.println("Testing device connections...");
            Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
            Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");
        }
        delay(1000);
    }

    void Calibrate(int _totalCalibrationTime)
    {
        // Use interrput pins to get readings from the accel, gyro, and mag

        // Compute vector magnitude for accelerometer, and figure out coefficient to make it equal to 9.807 m/s^2

        // Compute gyro drift rate here, make that as an offset

        // Calibrate magnetometer (move it arround so that we can get the proper offset)

        // Write the calibration values to the gyro, accel, and magnetometer to EEPROM (or flash memeory)
    }

    // This is the final function we want, we just want our current x, y, and z rotation
    float GetRotation()
    {
        // rotation = sin(magData[0] / magData[1]);

        // rotation = atan(magData[abs(orientation.y)] / magData[abs(orientation.x)]);

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
        // float yaw = atan(accelData[orientation.z] / sqrt(pow(accelData[orientation.x], 2) + pow(accelData[orientation.y], 2))) * RADS_TO_DEGS;
// 
        // Serial.println(String(atan(magData[abs(orientation.y)] / magData[abs(orientation.x)]) * RADS_TO_DEGS) + "\t" + \
        //                String(atan(magData[abs(orientation.z)] / magData[abs(orientation.y)]) * RADS_TO_DEGS) + "\t");
        // Serial.println(String(atan(magData[abs(orientation.y)] / magData[abs(orientation.x)]) * RADS_TO_DEGS) + "\t" + \
        //                String(atan(magData[abs(orientation.z)] / magData[abs(orientation.y)]) * RADS_TO_DEGS) + "\t");
    }

    void SyncronizeRotation()
    {
        // Get readings from accelerometer

        // Get readings from gyro

        // Get readings from magnetometer

        // Use the fact that the sum of the acceleration vector is equal to 9.8 (assuming that we aren't moving);=
    }

    void Update()
    {
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

        // offsets for mag
        mx += -5;
        my += -62;
        mz += 47;

        // offsets for gyro


        gx += 25;
        gy += -54;
        gz += 9;

        // Convert data into actual numbers
        // Accel -
        // Gyro - Range is +- 2g | 
        // Mag - Range is +- 8 guass = +-4096 | so one raw is equal to (8 / 4096.0) guass
        magData[0] = mx * MAG_RAW_TO_GUASS;
        magData[1] = my * MAG_RAW_TO_GUASS;
        magData[2] = mz * MAG_RAW_TO_GUASS;

        accelData[0] = ax * ACCEL_RAW_TO_G;
        accelData[1] = ay * ACCEL_RAW_TO_G;; 
        accelData[2] = az * ACCEL_RAW_TO_G;

        gyroData[0] = gx * GYRO_RAW_TO_DEG_PER_SEC;
        gyroData[1] = gy * GYRO_RAW_TO_DEG_PER_SEC;
        gyroData[2] = gz * GYRO_RAW_TO_DEG_PER_SEC;

        // offsets
        // gyroData[0] -= -1.54377548601399;
        // gyroData[1] -= 3.29122583916084;
        // gyroData[2] += 0.54718191958042;
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

        // gyro: -160.9132080078 26.5391845703 -74.7651977539 time:46526
        // gyroData[orientation.x] += 160.9132080078 / (46526 / 1000);
        // gyroData[orientation.y] += -26.5391845703 / (46526 / 1000);
        // gyroData[orientation.z] += 74.7651977539 / (46526 / 1000);
        
        // Median filter?

        // Rotate

        // RotateVector3D(&magData[orientation.x], &magData[orientation.y], &magData[orientation.z], theta, phi, psi);

        if(previousTime == -1) // This makes it so that we ignore the first few attempts
        {
            previousTime = currentTime;
            return;
        }

        accelTheta = atan(accelData[orientation.x] / sqrt(pow(accelData[orientation.z], 2) + pow(accelData[orientation.y], 2)));
        
        // This makes is so that instead of the domain being [-90, 90], we increase it to [-90, 270]
        if(accelData[orientation.z] < 0){
            accelTheta = PI - accelTheta;
        }
        //  * RADS_TO_DEGS;
        accelPhi = atan(accelData[orientation.y] / sqrt(pow(accelData[orientation.z], 2) + pow(accelData[orientation.x], 2)));
        // This makes is so that instead of the domain being [-90, 90], we increase it to [-90, 270]
        if(accelData[orientation.z] < 0){
            accelPhi = PI - accelPhi;
        }
        
        accelTheta *= RADS_TO_DEGS;
        accelPhi *= RADS_TO_DEGS;

        gyroTheta += gyroData[orientation.x] * deltaTime;
        gyroPhi += gyroData[orientation.y] * deltaTime;
        gyroPsi += gyroData[orientation.z] * deltaTime;
        
        float alpha = 0.05;
        
        theta = (theta + (gyroData[orientation.x] * deltaTime)) * (1- alpha) + accelTheta * alpha;
        phi = (phi + (gyroData[orientation.y] * deltaTime)) * (1- alpha) + accelPhi * alpha;


        previousTime = currentTime;
        // If this is the first time we have updated the sensor values then skip the rest of this function
        if (previousUpdateTime == -1)
            return;
    }

    void ToString()
    {
    }

    void PrintData()
    {
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

    void PrintMag()
    {
        Serial.print("Mag: ");
        Serial.print(magData[orientation.x]);
        Serial.print(" ");
        Serial.print(magData[orientation.y]);
        Serial.print(" ");
        Serial.print(magData[orientation.z]);
        Serial.print(" ");
        Serial.println(sqrt(magData[0] * magData[0] + magData[1] * magData[1] + magData[2] * magData[2]));
        Serial.println();
    }

    void PrintAccel()
    {
        Serial.print("Accel: ");
        Serial.print(accelData[orientation.x]);
        Serial.print(" ");
        Serial.print(accelData[orientation.y]);
        Serial.print(" ");
        Serial.print(accelData[orientation.z]);
        Serial.println();
    }

    void PrintGyro()
    {
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
protected:
    int16_t mx, my, mz;
    int16_t gx, gy, gz;
    int16_t ax, ay, az;

    float ACCEL_RAW_TO_G;
    float GYRO_RAW_TO_DEG_PER_SEC;
    float MAG_RAW_TO_GUASS;




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