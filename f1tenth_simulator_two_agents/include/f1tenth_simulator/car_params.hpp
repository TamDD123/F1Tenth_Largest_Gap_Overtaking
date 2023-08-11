#pragma once

namespace racecar_simulator {

struct CarParams {
    double wheelbase;
    double friction_coeff;
    double h_cg; // height of car's CG
    double l_f; // length from CG to front axle
    double l_r; // length from CG to rear axle
    double cs_f; // cornering stiffness coeff for front wheels
    double cs_r; // cornering stiffness coeff for rear wheels
    double mass;
    double Iz; // moment of inertia about z axis from CG
    double Cm1;
    double Cm2;
    double Cm3;
    double B_f;
    double C_f;
    double D_f;
    double B_r;
    double C_r;
    double D_r;
    double max_speed;
    double max_steering_angle;
    double max_steering_vel;
    double max_accel;
    double max_decel;
};

}
