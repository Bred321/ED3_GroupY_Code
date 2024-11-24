#include "agv_kinematics.hpp"


#define L_val 1
#define d_val 1


struct calculate_forward_kinematics(double v1_in, double v2_in, double v3_in, double v4_in){
    Forward_Output Forward_Result;
    Forward_Result.xm_dot = 0.25 * (v1_in + v2_in + v3_in + v4_in);
    Forward_Result.ym_dot = 0.25 * (v2_in + v4_in - v1_in - v3_in);
    Forward_Result.phi_dot = (0. 25 / (L_val + d_val)) * (v3_in + v4_in - v1_in - v2_in);
    return Forward_Result
}


struct calculate_inverse_kinematics(double xm_dot_in, double ym_dot_in, double phi_dot_in){
    Inverse_Output Inverse_Result;
    Inverse_Result.v1 = xm_dot_in - ym_dot_in - phi_dot_in * (L_val + d_val);
    Inverse_Result.v2 = xm_dot_in + ym_dot_in - phi_dot_in * (L_val + d_val);
    Inverse_Result.v3 = xm_dot_in - ym_dot_in + phi_dot_in * (L_val + d_val);
    Inverse_Result.v4 = xm_dot_in + ym_dot_in + phi_dot_in * (L_val + d_val);
    return Inverse_Result
}