#include <Arduino.h>
#include "motor_control.hpp"
#include "encoder_reading.hpp"


double angle_reading = 0;
double speed_reading = 0;
double deltaT = 0;
long t = 0;
long t_prev = 0;


double calculate_time(){
    // Calculate the delta T
    t = micros();
    deltaT = ((double)(t - t_prev))/1.0e6;
    t_prev = t;
    return t_prev;
}

void setup(){
   Init_Motor();
   Init_Encoder();
   Serial.begin(115200);
}


void loop(){
    // Drive the motor at full speed
    Run_Max_Speed();

    angle_reading = Get_Angle();
    Serial.print("Angle result: ");
    Serial.println(angle_reading);

    // Velocity reading
    double delta_t_reading = calculate_time();
    speed_reading = Get_Speed(delta_t_reading);
    Serial.print("Speed result: ");
    Serial.println(speed_reading);

    // Delay 1000s
    delay(1000);

}