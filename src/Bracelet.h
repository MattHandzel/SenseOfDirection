#include "Arduino.h"

#define TAU PI * 2

class VibrationMotor
{
private:
    // Creates varaibles for the vibration motors position (in rads),
    // the length of the "arc" it is going to span
    float m_position, m_arcLength = TAU;
    int m_pin;

public:
    // Variable for current power in case we want to read that
    float m_currentPower;

    // Sets the inputted pin to ouput and sets the position
    // of the motor to the inputted position
    VibrationMotor(int pin, float position) : m_position{position}, m_pin{pin}
    {
        pinMode(pin, OUTPUT);
    }

    VibrationMotor(int pin, float position, float arcLength) : m_pin{pin}, m_position{position}, m_arcLength{arcLength}
    {
        pinMode(pin, OUTPUT);
    }

    // Set what power to set the motor to
    void SetPower(float power)
    {
        power = (power > 1) ? 1 : power;
        analogWrite(this->m_pin, round(power * 1024));
        this->m_currentPower = power;
    }

    // Set the length of the arc (in case they want to change it later)
    void SetArcLength(float arcLength)
    {
        this->m_arcLength = arcLength;
    }

    // Set the position (in case they want to change it later)
    void SetPosition(float position)
    {
        this->m_position = position;
    }

    // This function will calculate how much power to send to the motor
    // based on the angle given
    void SetPowerBasedOnAngle(float angle)
    {
        // This function makes it so that angle
        // stays between 0 and 2pi
        while (angle > (PI + this->m_position))
        {
            angle -= TAU;
        }
        while (angle < (-PI + this->m_position))
        {
            angle += TAU;
        }

        float power = -square((angle - this->m_position) / (m_arcLength / 2)) + 1;

        power = (power > 0) ? power : 0;
        this->SetPower(power);
    }
};

class Bracelet
{
    int m_nMotors = 4;

    VibrationMotor *m_motors[4];

    int AVALIABLE_PINS[4] = {A0, A1, A2, A3};

    float m_kARCLENGTH = TAU / 4;

public:
    Bracelet()
    {
    }

    void GenerateMotors()
    {
        for (int i = 0; i < this->m_nMotors; i++)
        {
            this->m_motors[i] = new VibrationMotor(this->AVALIABLE_PINS[i], (TAU / m_nMotors) * i, this->m_kARCLENGTH);
        }
    }

    void SetAngle(float angle)
    {
        for (int i = 0; i < this->m_nMotors; i++)
        {
            this->m_motors[i]->SetPowerBasedOnAngle(angle);
        }
    }
};