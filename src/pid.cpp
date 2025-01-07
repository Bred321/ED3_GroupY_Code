#include "pid.hpp"
#include <cmath>
#include "encoder_reading.hpp"
#include "motor_control.hpp"

PID_CLASS::PID_CLASS(double kp, double kd, double ki, int motor) : Kp(kp), Ki(ki), Kd(kd),Motor(motor) { }


void PID_CLASS::calculate()
{
  // Read the feedback data of motor current speed (RPM)
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
  // Sometimes integration is NAN
  if(std::isnan(integration))
  {
    integration = 0;
  }
  // Calculate error
  error = inputSpeed - actualSpeed;
  //http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-tuning-changes/
  integration = integration + Ki*error;
  //http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-derivative-kick/
  double dInput = actualSpeed - last_actual_speed;
  
  // PID calculation
  u = (Kp*error) - Kd*dInput + integration;
  last_actual_speed = actualSpeed;

  // Drive the motor using PID outputs
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

// Set the target speed to do PID
void PID_CLASS::set_input(double input)
{
  this->inputSpeed = input;
}

double PID_CLASS::get_output()
{
  return this->u;
}
// Reset all PID related variables
void PID_CLASS::reset_PID()
{
  this->inputSpeed = 0;
  this->error_prev = 0;
  this->actualSpeed = 0;
  this->last_actual_speed = 0;
  this->u = 0;
  this->integration = 0;
}
// Set new PID parameters - Kp Kd Ki
void PID_CLASS::set_PID(double kp, double kd, double ki)
{
  this->Kp = kp;
  this->Kd = kd;
  this->Ki = ki;
}
// Conduct PID calculation process
void PID_CLASS::do_PID()
{
  if(!moving)
  {
    if(this->last_actual_speed != 0) this->reset_PID();
    return;
  }
  this->calculate();
}


