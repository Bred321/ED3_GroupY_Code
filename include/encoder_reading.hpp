#include <Arduino.h>
#ifndef ENC_READ_H
#define ENC_READ_H
#include "SimpleKalmanFilter.h"
extern volatile long int cnt1, cnt2, cnt3, cnt4; // Volatile as it changed during interrupt
extern double th1, th2, th3, th4;             // Position angle in degrees
extern volatile long int cnt1_prev, cnt2_prev, cnt3_prev, cnt4_prev;
extern double actual_speed1, actual_speed2, actual_speed3, actual_speed4;
void readEncoder1();

void Init_Encoder();

double Get_Angle();

void Get_Speed(double deltaT);

#endif