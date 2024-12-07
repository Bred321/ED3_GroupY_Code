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


PID_CLASS motor3(2, 0.01, 1.1, MOTOR3);
PID_CLASS motor4(2, 0.01, 1.1, MOTOR4);


void setup(){
   Init_Encoder();
   esp_now_setup();
   Serial.begin(115200);
 
   delay(1000);
}


void loop(){
    t = millis();
   
    
    motor3.set_input(80);
    motor4.set_input(-80);
    

    // digitalWrite(25, HIGH);
    // digitalWrite(26, LOW);
    
    if (millis() > next_PID) {
    deltaT = ((double)(t - t_prev))/1.0e3;
    t_prev = t;
    Get_Speed(deltaT);
    Serial.print(">Setpoint3: ");
    Serial.println(motor3.inputSpeed);
    Serial.print(">Speed result3: ");
    Serial.println(actual_speed3);
    Serial.print(">Setpoint4: ");
    Serial.println(motor4.inputSpeed);
    Serial.print(">Speed result4: ");
    Serial.println(actual_speed4);
    motor3.calculate();
    motor4.calculate();
    next_PID += PID_interval;
    // Serial.print(">deltaT: ");
    // Serial.println(deltaT);
    drive_motor(0,0,motor3.get_output(),motor4.get_output());
  }
   
    
    // Encoder reading
    angle_reading = Get_Angle();
    
    // Serial.print(">DeltaT: ");
    // Serial.println(deltaT);
    
    
    // delay(10);
   
    
    // Run_Max_Speed();
    // esp_now_send_message();

}