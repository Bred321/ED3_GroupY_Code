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

// PID parameters
double y = 0,u = 0;
int dir = 0, pwmPID = 0;



// Variables for PID controllers
double error = 0, error_prev = 0;
double speed_input= 100;
double integration = 0.0, derivative = 0;

//PID constants
double Kp = 0.05;
double Kd = 0; //0.0005
double Ki = 0; //0.5

long next_PID = 0;

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
   esp_now_setup();
   Serial.begin(115200);
   delay(2000);
}

// void PID()
// {
    
//     if(isnan(integration))
//     {
//       integration = 0;
//     }
//     error = speed_input - actual_speed;
//     integration = integration + error;
//     derivative = error-error_prev;
//     // PID calculation
//     u = (Kp*error) + (Kd*derivative) + (Ki*integration);
//     y = u;//Map the u value to 0-255
//     error_prev = error;
// }


// void setMotor(int dir, int pwmVal){
  
//   if(dir == 1){
//     analogWrite(25,0);
//     analogWrite(26,pwmVal);
//   }
//   else if(dir == -1)
//   {
//     analogWrite(25,pwmVal);
//     analogWrite(26,0);
//   }
// //   else 
// //   {
// //     digitalWrite(in1,LOW);
// //     digitalWrite(in2,LOW);
// //     pwmVal = 0;
// //   }
 
  
// }

// void drive_motor()
// {
//     if(u>0)
//     {
//       dir = 1;
//     }
//     else if(u<0)
//     {
//       dir = -1;
//     }

//     pwmPID = (int) fabs(y);
//     if(pwmPID > 255)
//     {
//       pwmPID = 255;
//     }
//     if(pwmPID < 30 )
//     {
//       pwmPID = 30;
//     }
//   setMotor(dir, pwmPID);
// }


void loop(){
    PID_CLASS leftwheel(1,0,0,MOTOR1);
    leftwheel.set_input(150);
    calculate_time();
    // digitalWrite(25, HIGH);
    // digitalWrite(26, LOW);
    Get_Speed(deltaT);
    if (millis() > next_PID) {
    leftwheel.calculate();
    next_PID += PID_interval;
  }
    // drive_motor();
    // Encoder reading
    angle_reading = Get_Angle();
    Serial.print(">Speed result: ");
    Serial.println(actual_speed1);
    Serial.print(">DeltaT: ");
    Serial.println(deltaT);
    // Run_Max_Speed();
    // esp_now_send_message();

}