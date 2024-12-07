#include <Arduino.h>
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
//============================================================
void Init_Motor();
//============================================================
void Send_PWM(int, int, double, int);

//============================================================
void Run_Motor();

void Run_Max_Speed();
#endif