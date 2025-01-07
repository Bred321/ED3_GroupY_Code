#include <Arduino.h>
#include "motor_control.hpp"
// FW is LPWM and BW is RPWM
#define MOT1_FW 22
#define MOT1_BW 23
#define MOT2_FW 4 
#define MOT2_BW 14
#define MOT3_FW 33
#define MOT3_BW 32
#define MOT4_FW 25
#define MOT4_BW 26
int moving = 0;

// Initialize all the output pins for motors control
void Init_Motor()
{
    pinMode(MOT1_FW, OUTPUT);
    pinMode(MOT1_BW, OUTPUT);
    pinMode(MOT2_FW, OUTPUT);
    pinMode(MOT2_BW, OUTPUT);
    pinMode(MOT3_FW, OUTPUT);
    pinMode(MOT3_BW, OUTPUT);
    pinMode(MOT4_FW, OUTPUT);
    pinMode(MOT4_BW, OUTPUT);
}
// Run all motors at maximum speed using PWM
void Run_Max_Speed(){
  analogWrite(MOT1_FW, 255);
  analogWrite(MOT1_BW, 0);
  analogWrite(MOT2_FW, 255);
  analogWrite(MOT2_BW, 0);
  analogWrite(MOT3_FW, 255);
  analogWrite(MOT3_BW, 0);
  analogWrite(MOT4_FW, 255);
  analogWrite(MOT4_BW, 0);
}
void Send_PWM(int pin, int pwm)
{
  for(int i = 0; i <= pwm; i++)
  {
    analogWrite(pin,i);
  }
}
// Set the motor speeds with according direction for a single motor
void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == 1) { 
      if      (reverse == 0) { Send_PWM(MOT1_FW, spd); Send_PWM(MOT1_BW, 0); }
      else if (reverse == 1) { Send_PWM(MOT1_FW, 0); Send_PWM(MOT1_BW, spd); }
    }
    else if (i == 2) {
      if      (reverse == 0) { analogWrite(MOT2_FW, spd); analogWrite(MOT2_BW, 0); }
      else if (reverse == 1) { analogWrite(MOT2_FW, 0); analogWrite(MOT2_BW, spd); }
    }
    else if (i == 3) {
      if      (reverse == 0) { analogWrite(MOT3_FW, spd); analogWrite(MOT3_BW, 0); }
      else if (reverse == 1) { analogWrite(MOT3_FW, 0); analogWrite(MOT3_BW, spd); }
    }
    else{
      if      (reverse == 0) { analogWrite(MOT4_FW, spd); analogWrite(MOT4_BW, 0); }
      else if (reverse == 1) { analogWrite(MOT4_FW, 0); analogWrite(MOT4_BW, spd); }
    }
  }
// Set the motor speeds with according direction for all motors
void drive_motor(int spd1, int spd2, int spd3, int spd4)
{

    setMotorSpeed(1, spd1);
    setMotorSpeed(2, spd2);
    setMotorSpeed(3, spd3);
    setMotorSpeed(4, spd4);
}