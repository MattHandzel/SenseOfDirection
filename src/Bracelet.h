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
        Serial.println("setting power of " + String(m_pin) + " to " + String(power));
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

        float power = ((angle - this->m_position) / (m_arcLength / 2));
        power = 1 - abs(power);

        power = (power > 0) ? power : 0;
        Serial.println("I am " + String(m_pin) + " and I am setting my power to " + String(power));
        this->SetPower(power);
    }
};
enum BraceletStatuses{
    CALIBRATING,
    ERROR,
    RUNNING
}; 
class Bracelet
{
    int m_nMotors = 4;

    VibrationMotor *m_motors[4];

    int AVALIABLE_PINS[4] = {A1, A2, A3, A6}; // A6 - East, A1 - N, A3 - West, A2 - South

    float m_kARCLENGTH = TAU / 2;

    BraceletStatuses status = CALIBRATING;

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

    // When the angle of the bracelet is set it will turn on all of the motors in the length
    void SetAngle(float angle)
    {
        for (int i = 0; i < this->m_nMotors; i++)
        {
            this->m_motors[i]->SetPowerBasedOnAngle(angle);
        }
    }

    void SetStatus(BraceletStatuses s){
        status = s;
    }

    void Update(){
        if(status == CALIBRATING){
            if((millis() % 2000) / 1000 == 0){
                for(VibrationMotor *motor : this->m_motors){
                    motor->SetPower(1);
                }
            }
            else{
                for(VibrationMotor *motor : this->m_motors){
                    motor->SetPower(0);
                }
            }
        }
        else if(status == ERROR){
            if((millis() % 500) / 250 == 0){
                for(VibrationMotor *motor : this->m_motors){
                    motor->SetPower(1);
                }
            }
            else{
                for(VibrationMotor *motor : this->m_motors){
                    motor->SetPower(0);
                }
            }
        }
    }
};