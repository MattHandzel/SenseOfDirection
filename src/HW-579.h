#include "Arduino.h"
#include "Wire.h"

#define HMC5883L_ADDRESS 0x1E

class HW579
{
    // Constructor, in here initialize the gyro, accel, and magnetometer
    HW579();

    void Calibrate(int _totalCalibrationTime)
    {
        // Use interrput pins to get readings from the accel, gyro, and mag

        // Compute vector magnitude for accelerometer, and figure out coefficient to make it equal to 9.807 m/s^2

        // Compute gyro drift rate here, make that as an offset

        // Calibrate magnetometer (move it arround so that we can get the proper offset)

        // Write the calibration values to the gyro, accel, and magnetometer to EEPROM (or flash memeory)
    }

    // This is the final function we want, we just want our current x, y, and z rotation
    double *GetRotation()
    {
    }

    void SyncronizeRotation()
    {
        // Get readings from accelerometer

        // Get readings from gyro

        // Get readings from magnetometer

        // Use the fact that the sum of the acceleration vector is equal to 9.8 (assuming that we aren't moving);=
    }
};