#ifndef PID_HPP
#define PID_HPP
#include <Arduino.h>
#include "motor_control.hpp"
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4
class PID_CLASS {
private:
    // PID gains
    double Kp;
    double Ki;
    double Kd;
    // Other PID parameters
    
    
    double actualSpeed {0.0};
    double integration {0.0};
    double derivative {0.0};
    int Motor=0;
public:
    double inputSpeed {0.0};
    double error = 0, error_prev = 0;
    double y {0};
    double u {0};
    PID_CLASS(double kp, double ki, double kd, int motor); // Constructor
    void calculate(); // Calculate PID output
    void set_input(double input);
    double get_output();
    void print_output();
};

#endif // PID_HPP