#pragma once

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"

namespace racecar_simulator {

class STKinematics {

public:

    static CarState update(
            const CarState start,
            double desired_speed,
            double desired_steer_ang,
            CarParams p,
            double dt);


    static CarState update_k(
            const CarState start,
            double accel,
            double steer_angle_vel,
            CarParams p,
            double dt);

    static double compute_accel(CarState & state, double desired_speed, CarParams & p);

    static double set_accel(double accel_, CarParams & p);

    static double compute_steer_vel(CarState & state, double desired_steer_ang, CarParams & p);

    static double set_steer_angle_vel(double steer_angle_vel_, CarParams & p);
};

}
