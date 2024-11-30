#include "pid.hpp"
#include <cmath>


// PID parameters
// double y = 0,u = 0;
// int dir = 0, pwmPID = 0;


// // Variables for PID controllers
// double error = 0, error_prev = 0;
// double speed_input= -150;
// double integration = 0.0, derivative = 0;

PID_CLASS::PID_CLASS(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd) { }


void PID_CLASS::calculate(double deltaT, double actualSpeed)
{
    
    if(std::isnan(integration))
    {
      integration = 0;
    }
    error = speed_input - actualSpeed;
    integration = integration + (error*deltaT);
    derivative = (error-error_prev)/deltaT;
    // PID calculation
    u = (Kp*error) + (Kd*derivative) + (Ki*integration);
    y = u/360*255;//Map the u value to 0-255
    error_prev = error;
}


void PID_CLASS::setMotor(int dir, int pwmVal){
  
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


void PID_CLASS::drive_motor()
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
