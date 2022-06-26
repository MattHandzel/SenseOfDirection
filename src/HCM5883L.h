#include "Arduino.h"
#include "Wire.h"
#include "Chip.h"

#define HMC5883L_ADDRESS 0x1E

#define CONFIGURATION_REGISTER_A_ADDRESS 0x00
#define CONFIGURATION_REGISTER_A_VALUE B01111100
#define CONFIGURATION_REGISTER_B_ADDRESS 0x01
#define CONFIGURATION_REGISTER_B_VALUE B00000000
#define MODE_REGISTER_ADDRESS 0x02
#define MODE_REGISTER_VALUE B00000000

class HCM5883L : public Chip
{
    /*  From the datasheet: Register list
       00 | Configuration Register A
       01 | Configuration Register B
       02 | Mode Register
       03 | Data Output X MSB Register
       04 | Data Output X LSB Register
       05 | Data Output Z MSB Register
       06 | Data Output Z LSB Register
       07 | Data Output Y MSB Register
       08 | Data Output Y LSB Register
       09 | Status Register
       10 | Identification Register A
       11 | Identification Register B
       12 | Identification Register C

    */

public:
    HCM5883L(int _verbose) : Chip(HMC5883L_ADDRESS, _verbose) {}

    void Init()
    {
        Wire.begin();
        /*
        Configuration Register A:
        0    |       0             0   |        0            0          0    |       0           0           0
        N/A  | Samples averaged (1-8)  | Ouput rate (0.75-75; 15 default)    | Measurement configuration bits
        */
        Write(CONFIGURATION_REGISTER_A_ADDRESS, CONFIGURATION_REGISTER_A_VALUE);

        /* Configuration Register B:
         0           0             0            |  0            0          0           0           0           0
         Gain configuration bits (0.88 Ga-8.1ga)|                                          N/A
        */
        Write(CONFIGURATION_REGISTER_B_ADDRESS, CONFIGURATION_REGISTER_B_VALUE);

        /* Mode Register:
         0                          0 0 0 0 0 0   |   0           0
        High speed I2C Enable |         N/A       |  Operating Mode (00-continuous, 01- single; 01/11 - idle)                                                          N/A
        */
        Write(MODE_REGISTER_ADDRESS, MODE_REGISTER_VALUE);

        // Delay 6 ms (as datasheet says)
        delay(6);
    }

    float *GetReadings()
    {
        Wire.beginTransmission(HMC5883L_ADDRESS);
        Wire.write(0x06);
        Wire.endTransmission();

        Wire.beginTransmission(HMC5883L_ADDRESS);
        Wire.requestFrom(HMC5883L_ADDRESS, 6);

        short x = Wire.read() << 8 | Wire.read();
        short y = Wire.read() << 8 | Wire.read();
        short z = Wire.read() << 8 | Wire.read();

        if (m_verbose)
        {
            Serial.println("x:\t" + String(x) + "\ty:\t" + String(y) + "\tz:\t" + String(z));
            // Serial.println(d.f);
        }
        Wire.endTransmission();
        Wire.beginTransmission(HMC5883L_ADDRESS);

        // TODO:: Fact check this to see if what im saying makes sense
        // Sets the device back to the original pointer as datasheet says
        Wire.write(0x03);
        Wire.endTransmission();
        float result[3] = {(float)x, (float)y, (float)z};
        return result;
    }
};