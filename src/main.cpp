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
  //Variable for current power in case we want to read that
  float m_currentPower;

  // Sets the inputted pin to ouput and sets the position
  // of the motor to the inputted position
  VibrationMotor(int pin, float position) : m_position{position}, m_pin{pin}
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
  void setPosition(float position){
    this->m_position = position;
  }

  // This function will calculate how much power to send to the motor
  // based on the angle given
  void setPowerBasedOnAngle(float angle)
  {
    
  }
};

class Bracelet{
  int m_nMotors = 4;

  VibrationMotor m_motors[4];

  int AVALIABLE_PINS[4] = {A0,A1,A2,A3};

  void generateMotors(){
    for(int i = 0; i < this->m_nMotors; i++){
      this->m_motors[i] = VibrationMotor(this->AVALIABLE_PINS[i], PHI / m_nMotors * i);
    }
  }


};

VibrationMotor motor0 = VibrationMotor(A0, 0);
VibrationMotor motor1 = VibrationMotor(A1, PHI / 4);
VibrationMotor motor2 = VibrationMotor(A2, PHI / 2);
VibrationMotor motor3 = VibrationMotor(A3, 3 * PHI / 2);

void setup(){
  Serial.begin(9600);
}


float currentAngle = 0;
void loop()
{

  motor0.setPowerBasedOnAngle(currentAngle);
  motor1.setPowerBasedOnAngle(currentAngle);
  motor2.setPowerBasedOnAngle(currentAngle);
  motor3.setPowerBasedOnAngle(currentAngle);

  delay(100);
  currentAngle += PI / 20;
}
