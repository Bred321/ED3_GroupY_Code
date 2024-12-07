#include "pid.hpp"
#include <cmath>
#include "encoder_reading.hpp"
#include "motor_control.hpp"

PID_CLASS::PID_CLASS(double kp, double kd, double ki, int motor) : Kp(kp), Ki(ki), Kd(kd),Motor(motor) { }


void PID_CLASS::calculate()
{
  switch(Motor){
    case 1:
    this->actualSpeed = actual_speed1;
    break;
    case 2:
    this->actualSpeed = actual_speed2;
    break;
    case 3:
    this->actualSpeed = actual_speed3;
    break;
    default:
    this->actualSpeed = actual_speed4;
    break;
  }

  if(std::isnan(integration))
  {
    integration = 0;
  }
  error = inputSpeed - actualSpeed;
  integration = integration + error;
  derivative = error-error_prev;
  // Serial.print("error: ");
  // Serial.println(error);
  // Serial.print("error: ");
  // Serial.println(error);
 
  // PID calculation
  u = (Kp*error) + (Kd*derivative) + (Ki*integration);
  // Serial.print("u: ");
  // Serial.println(u);
  error_prev = error;
  // Drive the motor
  switch(Motor){
    case 1:
    setMotorSpeed(1, u);
    break;
    case 2:
    setMotorSpeed(2, u);
    break;
    case 3:
    setMotorSpeed(3, u);
    break;
    default:
    setMotorSpeed(4, u);
    break;
  }
}


void PID_CLASS::set_input(double input)
{
  this->inputSpeed = input;
}

double PID_CLASS::get_output()
{
  return this->u;
}
void PID_CLASS::reset_PID()
{
  this->inputSpeed = 0;
  this->u = 0;
  this->integration = 0;
}
void PID_CLASS::set_PID(double kp, double kd, double ki)
{
  this->Kp = kp;
  this->Kd = kd;
  this->Ki = ki;
}


