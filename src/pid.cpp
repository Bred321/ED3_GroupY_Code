#include "pid.hpp"
#include <cmath>
#include "encoder_reading.hpp"
// PID parameters
// double y = 0,u = 0;
// int dir = 0, pwmPID = 0;


// // Variables for PID controllers
// double error = 0, error_prev = 0;
// double speed_input= -150;
// double integration = 0.0, derivative = 0;

PID_CLASS::PID_CLASS(double kp, double ki, double kd, int motor) : Kp(kp), Ki(ki), Kd(kd),Motor(motor) { }


void PID_CLASS::calculate()
{
  switch(Motor){
    case 1:
    actualSpeed = actual_speed1;
    break;
    case 2:
    actualSpeed = actual_speed2;
    break;
    case 3:
    actualSpeed = actual_speed3;
    break;
    default:
    actualSpeed = actual_speed4;
    break;
  }

  if(std::isnan(integration))
  {
    integration = 0;
  }
  error = inputSpeed - actualSpeed;
  integration = integration + error;
  derivative = error-error_prev;
  // PID calculation
  u = (Kp*error) + (Kd*derivative) + (Ki*integration);
  y = u/360*255;//Map the u value to 0-255
  error_prev = error;
}

void PID_CLASS::set_input(double input)
{
  this->inputSpeed = input;
}

