#include <Arduino.h>
#include <esp_now.h>
#include "WIFI.h"
#include "motor_control.hpp"
#include "encoder_reading.hpp"
#include "agv_kinematics.hpp"
#include "esp_now_sender.hpp"
#include "pid.hpp"
#define PID_rate 30
#define PID_interval 1000/PID_rate


// Variables for encoder value reading
double angle_reading = 0;
double deltaT = 0;
long t = 0;
long t_prev = 0;

long next_PID = 0;


void calculate_time(){
    // Calculate the delta T
    t = esp_timer_get_time();
    deltaT = ((double)(t - t_prev))/1.0e6;
    t_prev = t;
}
PID_CLASS leftwheel(2,0.01,1.1,MOTOR1);
void setup(){
   pinMode(25, OUTPUT);
   pinMode(26, OUTPUT);
   Init_Encoder();
   esp_now_setup();
   Serial.begin(115200);
 
   delay(1000);
}


void loop(){
    t = millis();
   
    
    leftwheel.set_input(80);
    

    // digitalWrite(25, HIGH);
    // digitalWrite(26, LOW);
    
    if (millis() > next_PID) {
    deltaT = ((double)(t - t_prev))/1.0e3;
    t_prev = t;
    Get_Speed(deltaT);
    Serial.print(">Setpoint: ");
    Serial.println(leftwheel.inputSpeed);
    Serial.print(">Speed result: ");
    Serial.println(actual_speed1);
    leftwheel.calculate();
    next_PID += PID_interval;
    Serial.print(">deltaT: ");
    Serial.println(deltaT);
    drive_motor(leftwheel.get_output(),0,0,0);
  }
   
    
    // Encoder reading
    angle_reading = Get_Angle();
    
    // Serial.print(">DeltaT: ");
    // Serial.println(deltaT);
    
    
    // delay(10);
   
    
    // Run_Max_Speed();
    // esp_now_send_message();

}