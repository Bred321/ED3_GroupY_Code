#include <Arduino.h>
#include "motor_control.hpp"

#define MOT1_FW 23
#define MOT1_BW 22
#define MOT2_FW 4 
#define MOT2_BW 14
#define MOT3_FW 32
#define MOT3_BW 33
#define MOT4_FW 26
#define MOT4_BW 25

#define MOT1_Channel 0 // MOT 1 channel

#define PWM_FREQ 10000 // PWM Frequency: 10kHz
#define PWM_RES 8      // PWM resolution 255

double MOT1_cmd = 0; // MOT1 command [-255; 255]

//============================================================
void Init_Motor()
{
    pinMode(MOT1_FW, OUTPUT);
    pinMode(MOT1_BW, OUTPUT);
    ledcSetup(MOT1_Channel, PWM_FREQ, PWM_RES);
}
//============================================================
void Send_PWM(int PINA, int PINB, double mot_cmd, int channel)
{
    // Reverse direction, if the mot_cmd is from [-255; 0]
    if (mot_cmd < 0)
    {
        ledcAttachPin(PINB, channel);
        ledcDetachPin(PINA);
        digitalWrite(PINA, LOW);
        ledcWrite(channel, abs(mot_cmd));
    }
    // Forward direction, if the mot_cmd is from [0; 255]; 
    else 
    {
        ledcAttachPin(PINA, channel);
        ledcDetachPin(PINB);
        digitalWrite(PINB, LOW);
        ledcWrite(channel, abs(mot_cmd));
    }
}

//============================================================


void Run_Max_Speed(){
  analogWrite(MOT1_FW, 255);
  analogWrite(MOT1_BW, 0);
}

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
      if      (reverse == 0) { analogWrite(MOT1_FW, spd); analogWrite(MOT1_BW, 0); }
      else if (reverse == 1) { analogWrite(MOT1_FW, 0); analogWrite(MOT1_BW, spd); }
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

void drive_motor(int spd1, int spd2, int spd3, int spd4)
{
    setMotorSpeed(1, spd1);
    setMotorSpeed(2, spd2);
    setMotorSpeed(3, spd3);
    setMotorSpeed(4, spd4);
}