#include <Arduino.h>
#include <stdio.h>

#define PHI PI * 2

class VibrationMotor
{
private:
  // Creates varaibles for the vibration motors position (in rads),
  // the length of the "arc" it is going to span
  float m_position, m_arcLength = PHI;
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
  void setPower(float power)
  {
    power = (power > 1) ? 1 : power;
    analogWrite(this->m_pin, round(power * 1024));
    this->m_currentPower = power;
  }

  // Set the length of the arc (in case they want to change it later)
  void setArcLength(float arcLength)
  {
    this->m_arcLength = arcLength;
  }

  // Set the position (in case they want to change it later)
  void setPosition(float position)
  {
    this->m_position = position;
  }

  // This function will calculate how much power to send to the motor
  // based on the angle given
  void setPowerBasedOnAngle(float angle)
  {
    // This function makes it so that angle
    // stays between 0 and 2pi
    while (angle > (PI + this->m_position))
    {
      angle -= PHI;
    }
    while (angle < (-PI + this->m_position))
    {
      angle += PHI;
    }

    float power = -square((angle - this->m_position) / (m_arcLength / 2)) + 1;

    power = (power > 0) ? power : 0;
    this->setPower(power);
  }
};

class Bracelet
{
  int m_nMotors = 4;

  VibrationMotor *m_motors[4];

  int AVALIABLE_PINS[4] = {A0, A1, A2, A3};

  float m_kARCLENGTH = PHI / 4;

public:
  Bracelet()
  {
  }

  void generateMotors()
  {
    for (int i = 0; i < this->m_nMotors; i++)
    {
      this->m_motors[i] = new VibrationMotor(this->AVALIABLE_PINS[i], (PHI / m_nMotors) * i, this->m_kARCLENGTH);
    }
  }

  void setAngle(float angle)
  {
    for (int i = 0; i < this->m_nMotors; i++)
    {
      this->m_motors[i]->setPowerBasedOnAngle(angle);
    }
  }
};

Bracelet b = Bracelet();

void setup()
{
  Serial.begin(9600);
  b.generateMotors();
  pinMode(3, OUTPUT);
}

float currentAngle = 0;
void loop()
{
  b.setAngle(currentAngle);
  currentAngle += PI / 16;
  delay(100);
  Serial.println(currentAngle);
}
