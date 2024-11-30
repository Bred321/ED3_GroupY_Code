#ifndef PID_HPP
#define PID_HPP

class PID_CLASS {
private:
    // PID gains
    double Kp;
    double Ki;
    double Kd;
    // Other PID parameters
    double y {0};
    double u {0};
    int dir {0}
    int pwmPID {0};
    double inputSpeed {-150};
    double integration {0.0};
    double derivative {0.0};
public:
    PID_CLASS(double kp, double ki, double kd); // Constructor
    void calculate(double error, double dt); // Calculate PID output
    void setMotor(int motor1, int motor2); // Set motor speeds
    void driveMotor(); // Drive the motors
};

#endif // PID_HPP