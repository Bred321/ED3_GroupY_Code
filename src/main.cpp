#include "Arduino.h"
#include "motor_control.hpp"
#include "encoder_reading.hpp"


double angle_reading = 0;
double speed_reading = 0;
double deltaT = 0;
long t = 0;
long t_prev = 0;


long calculate_time(){
    // Calculate the delta T
    t = micros();
    deltaT = ((double)(t - t_prev)) / 1.0e6;
    t_prev = t;
    return deltaT;
}

void setup(){
   pinMode(25, OUTPUT);
   pinMode(26, OUTPUT);
   Init_Encoder();
   Serial.begin(9600);
}


void loop(){
    digitalWrite(25, HIGH);
    digitalWrite(26, LOW);

    // Encoder reading
    angle_reading = Get_Angle();
    Serial.print("Angle result: ");
    Serial.println(angle_reading);

    double delta_t_reading = calculate_time();
    speed_reading = Get_Speed(delta_t_reading);
    Serial.print("Speed result: ");
    Serial.println(speed_reading);

    delay(1000);

}