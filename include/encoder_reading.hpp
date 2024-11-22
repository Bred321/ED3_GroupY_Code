#include <Arduino.h>

#define ENC1_A 33    // MOT 1A
#define ENC1_B 32   // MOT 1B
#define ENC_RES 330 // Encoder resolution*Gearbox ratio: 11*30

extern volatile long int cnt1; // Volatile as it changed during interrupt
extern double th1;             // Position angle in degrees
extern volatile long int count_a_prev;
extern volatile long int count_a;
extern double raw_speed ;
extern double actual_speed ;

//================================================================================
void readEncoder1();

void Get_Speed(double deltaT);
    
void Init_Encoder();

double Get_Angle();


