#include "Arduino.h"
#include "Wire.h"

const int MPU_addr = 0x68; // I2C address of the MPU-6050

class MPU6050
{

    int16_t acc[3];
    int16_t gyro[3];
    int16_t temp;

    int32_t offsets[6];

    // Make this the target offsets
    int32_t targets[6] = {0, 0, 0, 0, 0, 0};

    int32_t errors[6];

    int16_t LOWER_THRESHOLD = 400;

    int32_t totalAcc[3];
    int32_t totalGyro[3];

public:
    void Initialize()
    {
        Wire.begin();
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x6B); // PWR_MGMT_1 register
        Wire.write(0);    // set to zero (wakes up the MPU-6050)
        Wire.endTransmission(true);
    }

    void GetReadings()
    {
        // TODO:: Make it so that we get the delta time inside this function so that we can more accurately measure the change in gyro over time
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr, 14, true);           // request a total of 14 registers
        this->acc[0] = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        this->acc[1] = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        this->acc[2] = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        this->temp = Wire.read() << 8 | Wire.read();    // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
        this->gyro[0] = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        this->gyro[1] = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        this->gyro[2] = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    }

    void CleanReadings()
    {
        // Add the offset to the readings
        this->acc[0] = this->acc[0] + offsets[0];
        this->acc[1] = this->acc[1] + offsets[1];
        this->acc[2] = this->acc[2] + offsets[2];

        this->gyro[0] = this->gyro[0] + offsets[3];
        this->gyro[1] = this->gyro[1] + offsets[4];
        this->gyro[2] = this->gyro[2] + offsets[5];

        // Add a thresholf for lower ambplitude readings
        this->acc[0] = abs(this->acc[0]) > LOWER_THRESHOLD ? this->acc[0] : 0;
        this->acc[1] = abs(this->acc[1]) > LOWER_THRESHOLD ? this->acc[1] : 0;
        this->acc[2] = abs(this->acc[2]) > LOWER_THRESHOLD ? this->acc[2] : 0;

        // Guy has a lower threshold then acc because it is more precise
        this->gyro[0] = abs(this->gyro[0]) > LOWER_THRESHOLD / 2 ? this->gyro[0] : 0;
        this->gyro[1] = abs(this->gyro[1]) > LOWER_THRESHOLD / 2 ? this->gyro[1] : 0;
        this->gyro[2] = abs(this->gyro[2]) > LOWER_THRESHOLD / 2 ? this->gyro[2] : 0;

        this->totalAcc[0] += this->acc[0];
        this->totalAcc[1] += this->acc[1];
        this->totalAcc[2] += this->acc[2];

        this->totalGyro[0] += this->gyro[0];
        this->totalGyro[1] += this->gyro[1];
        this->totalGyro[2] += this->gyro[2];
    }

    void ToSerialRaw()
    {
        Serial.print("AcX = ");
        Serial.print(this->acc[0]);
        Serial.print(" | AcY = ");
        Serial.print(this->acc[1]);
        Serial.print(" | AcZ = ");
        Serial.print(this->acc[2]);
        Serial.print(" | GyX = ");
        Serial.print(this->gyro[0]);
        Serial.print(" | GyY = ");
        Serial.print(this->gyro[1]);
        Serial.print(" | GyZ = ");
        Serial.print(this->gyro[2]);
        Serial.print(" | temp = ");
        Serial.println(this->temp / 340.00 + 36.53);
    }

    void TotalToSerial()
    {
        Serial.print(" | AcX = ");
        Serial.print(this->totalAcc[0]);
        Serial.print(" | AcY = ");
        Serial.print(this->totalAcc[1]);
        Serial.print(" | AcZ = ");
        Serial.print(this->totalAcc[2]);
        Serial.print(" | GyX = ");
        Serial.print(this->totalGyro[0]);
        Serial.print(" | GyY = ");
        Serial.print(this->totalGyro[1]);
        Serial.print(" | GyZ = ");
        Serial.print(this->totalGyro[2]);
        Serial.println();
    }

    void ToSerial()
    {
        Serial.print("You have rotated ");
        Serial.print(this->GetRotation());
        Serial.println(" degrees");
    }

    void StartCalibration(int32_t nMillis, int16_t period)
    {
        for (int i = 0; i < nMillis / period; i++)
        {
            CalibrateSensor();
            delay(period);
        }

        offsets[0] /= nMillis / period;
        offsets[1] /= nMillis / period;
        offsets[2] /= nMillis / period;
        offsets[3] /= nMillis / period;
        offsets[4] /= nMillis / period;
        offsets[5] /= nMillis / period;
    }

    void CalibrateSensor()
    {
        this->GetReadings();
        errors[0] = targets[0] - this->acc[0];
        errors[1] = targets[1] - this->acc[1];
        errors[2] = targets[2] - this->acc[2];

        errors[3] = targets[3] - this->acc[3];
        errors[4] = targets[4] - this->acc[4];
        errors[5] = targets[5] - this->acc[5];

        offsets[0] += errors[0];
        offsets[1] += errors[1];
        offsets[2] += errors[2];
        offsets[3] += errors[3];
        offsets[4] += errors[4];
        offsets[5] += errors[5];
    }

    double GetRotation()
    {
        // This is the tested value for 360 degrees of rotation
        return (double)this->totalGyro[2] / 16696473 * 360;
    }
};