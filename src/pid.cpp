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
  integration = integration + Ki*error;
  // derivative = error-error_prev;
  double dInput = actualSpeed - last_actual_speed;
  
 
  // PID calculation
  // u = (Kp*error) + (Kd*derivative) + integration;
  u = (Kp*error) - Kd*dInput + integration;
  // error_prev = error;
  last_actual_speed = actualSpeed;

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
  this->error_prev = 0;
  this->actualSpeed = 0;
  this->last_actual_speed = 0;
  this->u = 0;
  this->integration = 0;
}
void PID_CLASS::set_PID(double kp, double kd, double ki)
{
  this->Kp = kp;
  this->Kd = kd;
  this->Ki = ki;
}
void PID_CLASS::do_PID()
{
  if(!moving)
  {
    if(this->last_actual_speed != 0) this->reset_PID();
    return;
  }
  this->calculate();
}


