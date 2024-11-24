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


struct Forward_Output calculate_forward_kinematics(double v1_in, double v2_in, double v3_in, double v4_in);
struct Inverse_Output calculate_inverse_kinematics(double xm_dot_in, double ym_dot_in, double phi_dot_in);


#endif // HEADER_HPP
