#ifndef HEADER_HPP
#define HEADER_HPP

struct Forward_Output {
    double xm_dot;
    double ym_dot;
    double phi_dot;
};


struct Inverse_Output {
    double v1;
    double v2;
    double v3;
    double v4;
};


struct calculate_forward_kinematics(double, double, double, double);
struct calculate_inverse_kinematics(double, double, double);


#endif // HEADER_HPP
