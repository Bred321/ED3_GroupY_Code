#ifndef PID_HPP
#define PID_HPP
#include <Arduino.h>
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
    double y {0};
    double u {0};
    double inputSpeed {-150};
    double actualSpeed {0.0};
    double integration {0.0};
    double derivative {0.0};
    double error = 0, error_prev = 0;
    int Motor=0;
public:
    PID_CLASS(double kp, double ki, double kd, int motor); // Constructor
    void calculate(); // Calculate PID output
    void set_input(double input);
};

#endif // PID_HPP