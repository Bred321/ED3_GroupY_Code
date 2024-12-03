#include "pid.hpp"
#include <cmath>
#include "encoder_reading.hpp"

PID_CLASS::PID_CLASS(double kp, double ki, double kd, int motor) : Kp(kp), Ki(ki), Kd(kd),Motor(motor) { }


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

  // if(std::isnan(integration))
  // {
  //   integration = 0;
  // }
  error = inputSpeed - actualSpeed;
  integration = integration + error;
  derivative = error-error_prev;
  Serial.print("error: ");
  Serial.println(error);
  Serial.print("error_prev1: ");
  Serial.println(error_prev);
  Serial.print("d: ");
  Serial.println(derivative);
  Serial.print("i: ");
  Serial.println(integration);
  // PID calculation
  u = (Kp*error) + (Kd*derivative) + (Ki*integration);
  // y = u/360*255;//Map the u value to 0-255
  y = u;
  error_prev = error;
  Serial.print("error_prev2: ");
  Serial.println(error_prev);
}

void PID_CLASS::set_input(double input)
{
  this->inputSpeed = input;
}

double PID_CLASS::get_output()
{
  return this->y;
}


