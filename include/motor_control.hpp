#include <Arduino.h>
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
//============================================================
void Init_Motor();
//============================================================
void Send_PWM(int, int, double, int);
void setMotorSpeed(int i, int spd);
//============================================================
void drive_motor(int spd1, int spd2, int spd3, int spd4);
void Run_Max_Speed();
#endif