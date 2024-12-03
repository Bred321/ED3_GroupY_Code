#include <Arduino.h>
#include "encoder_reading.hpp"

#define ENC1_A 33    // MOT 1A
#define ENC1_B 32  // MOT 1B
#define ENC_RES 330 // Encoder resolution*Gearbox ratio: 11*30

volatile long int cnt1 = 0, cnt2 = 0, cnt3 = 0, cnt4 = 0; // Volatile as it changed during interrupt
double th1 = 0;             // Position angle in degrees
volatile long int cnt1_prev = 0, cnt2_prev = 0, cnt3_prev = 0, cnt4_prev = 0;
double actual_speed1 = 0, actual_speed2 = 0, actual_speed3 = 0 ,actual_speed4 = 0;

//================================================================================
void readEncoder1()
{
    int b = digitalRead(ENC1_B);
    cnt1 = (b > 0) ? (cnt1 + 1) : (cnt1 - 1);
    
    
}
void readEncoder2()
{
    int b = digitalRead(ENC1_B);
    cnt2 = (b > 0) ? (cnt2 + 1) : (cnt2 - 1);
    
    
}
void readEncoder3()
{
    int b = digitalRead(ENC1_B);
    cnt3 = (b > 0) ? (cnt3 + 1) : (cnt3 - 1);
    
    
}
void readEncoder4()
{
    int b = digitalRead(ENC1_B);
    cnt4 = (b > 0) ? (cnt4 + 1) : (cnt4 - 1);
    
    
}

void Init_Encoder()
{
    pinMode(ENC1_A, INPUT);
    pinMode(ENC1_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC1_A), readEncoder1, RISING);
}

double Get_Angle()
{
    th1 = cnt1 * 360 / ENC_RES; // Conversion between encoder count and degree
    return th1;
}

void Get_Speed(double deltaT){
    // Convert raw to real speed
    actual_speed1 = (((cnt1 - cnt1_prev) / deltaT)/360) * 60;
    cnt1_prev = cnt1;
    actual_speed2 = (((cnt2 - cnt2_prev) / deltaT)/360) * 60;
    cnt2_prev = cnt2;
    actual_speed3 = (((cnt3 - cnt3_prev) / deltaT)/360) * 60;
    cnt3_prev = cnt1;
    actual_speed4 = (((cnt4 - cnt4_prev) / deltaT)/360) * 60;
    cnt4_prev = cnt4;
}