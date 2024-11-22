#include <Arduino.h>
#include "motor_control.hpp"
#include "encoder_reading.hpp"


double angle_reading = 0;
double speed_reading = 0;
double deltaT = 0;
long t = 0;
long t_prev = 0;


void calculate_time(){
    // Calculate the delta T
    t = micros();
    deltaT = ((double)(t - t_prev))/1.0e6;
    t_prev = t;
}

void setup(){
   pinMode(25, OUTPUT);
   pinMode(26, OUTPUT);
   Init_Encoder();
   Serial.begin(115200);
}


void loop(){
    calculate_time();
    digitalWrite(25, HIGH);
    digitalWrite(26, LOW);
    Get_Speed(deltaT);
    // Encoder reading
    angle_reading = Get_Angle();
    Serial.print("Angle result: ");
    Serial.println(angle_reading);

    
   
    Serial.print("Speed result: ");
    Serial.println(actual_speed);

    delay(300);

}