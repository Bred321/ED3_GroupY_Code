#include <Arduino.h>
#include "motor_control.hpp"
#include "encoder_reading.hpp"


double angle_reading = 0;
double deltaT = 0;
long t = 0;
long t_prev = 0;
double y = 0,u = 0;
int dir = 0, pwmPID = 0;




double error = 0, error_prev = 0;
double speed_input= 150;
double integration = 0.0, derivative = 0;

//PID constants
double Kp = 1.57;
double Kd = 0.0005; //0.01
double Ki = 0.6; //0.5


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

void PID(double deltaT)
{
    
    if(isnan(integration))
    {
      integration = 0;
    }
    error = speed_input - actual_speed;
    integration = integration + (error*deltaT);
    derivative = (error-error_prev)/deltaT;
    // PID calculation
    u = (Kp*error) + (Kd*derivative) + (Ki*integration);
    y = u/360*255;//Map the u value to 0-255
    error_prev = error;
}


void setMotor(int dir, int pwmVal){
  
  if(dir == 1){
    analogWrite(25,0);
    analogWrite(26,pwmVal);
  }
  else if(dir == -1)
  {
    analogWrite(25,pwmVal);
    analogWrite(26,0);
  }
//   else 
//   {
//     digitalWrite(in1,LOW);
//     digitalWrite(in2,LOW);
//     pwmVal = 0;
//   }
 
  
}

void drive_motor()
{
    if(u>0)
    {
      dir = 1;
    }
    else if(u<0)
    {
      dir = -1;
    }

    pwmPID = (int) fabs(y);
    if(pwmPID > 255)
    {
      pwmPID = 255;
    }
    if(pwmPID < 30 )
    {
      pwmPID = 30;
    }
  setMotor(dir, pwmPID);
}


void loop(){
    calculate_time();
    // digitalWrite(25, HIGH);
    // digitalWrite(26, LOW);
    Get_Speed(deltaT);
    PID(deltaT);
    drive_motor();
    // Encoder reading
    angle_reading = Get_Angle();
    Serial.print("Angle result: ");
    Serial.println(angle_reading);

    
   
    Serial.print("Speed result: ");
    Serial.println(actual_speed);

    Serial.print("error: ");
    Serial.println(error);

    Serial.print("i: ");
    Serial.println(integration);

    Serial.print("d: ");
    Serial.println(derivative);

    delay(300);

}