#include "Arduino.h"
#include "Wire.h"
class Chip
{
public:
    Chip(int _address, int _verbose) : m_address{_address}, m_verbose(_verbose){};

    /**
     * @brief Writes the value into the register for a device with a certain addres
     *
     * @param _address
     * @param _reg
     * @param _value
     */
    void Write(int _reg, int _value)
    {
        Wire.beginTransmission(m_address);
        Wire.write(_reg);
        Wire.write(_value);
        Wire.endTransmission();
    }

    byte *Read(int _regAddress, int num)
    {
        byte *data = new byte[num];
        Wire.beginTransmission(m_address);
        Wire.write(_regAddress);
        Wire.endTransmission();

        Wire.beginTransmission(m_address);
        Wire.requestFrom(m_address, num);

        int i = 0;
        while (Wire.available())
        {
            data[i] = Wire.read();
            i++;
        }
        Wire.endTransmission();
        return data;
    }

    float *GetReadings();

    void PrintReadings()
    {
        float *readings = GetReadings();
        for (int i = 0; i < sizeof(readings) / sizeof(float); i++)
        {
            Serial.print(readings[i]);
            Serial.print("\t");
        }
        Serial.println();
    }

protected:
    int m_verbose;
    int m_address;
};