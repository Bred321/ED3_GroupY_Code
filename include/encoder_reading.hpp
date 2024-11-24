#include <Arduino.h>
#ifndef ENC_READ_H
#define ENC_READ_H
extern volatile long int cnt1; // Volatile as it changed during interrupt
extern double th1;             // Position angle in degrees
extern volatile long int count_a_prev;
extern volatile long int count_a;
extern double raw_speed;
extern double actual_speed;
void readEncoder1();

void Init_Encoder();

double Get_Angle();

void Get_Speed(double deltaT);

#endif