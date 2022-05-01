#include <Arduino.h>
#include <stdio.h>

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
      angle -= TAU;
    }
    while (angle < (-PI + this->m_position))
    {
      angle += TAU;
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

  float m_kARCLENGTH = TAU / 4;

public:
  Bracelet()
  {
  }

  void generateMotors()
  {
    for (int i = 0; i < this->m_nMotors; i++)
    {
      this->m_motors[i] = new VibrationMotor(this->AVALIABLE_PINS[i], (TAU / m_nMotors) * i, this->m_kARCLENGTH);
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
  pinMode(A5, INPUT_PULLUP);
}

float currentAngle = 0, speed = 0;
long startTime, endTime;
int timePerLoop = 50;
void loop()
{
  startTime = millis();
  b.setAngle(currentAngle);

  speed = (min((float)analogRead(A5) / 220, 1));

  currentAngle += PI / 2 * speed;

  if (currentAngle > 6)
  {
    currentAngle -= TAU;
  }

  endTime = millis();
  while (endTime - startTime < timePerLoop)
  {
    delay(1);
    endTime = millis();
  }
}
