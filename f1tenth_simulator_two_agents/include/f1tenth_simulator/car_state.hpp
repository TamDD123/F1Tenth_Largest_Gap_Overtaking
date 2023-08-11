#pragma once

namespace racecar_simulator {

struct CarState {
    double x; // x position
    double y; // y position
    double theta; // orientation, anticlockwise from 0 to 2Pi in radian
    double velocity_x;
    double velocity_y;
    double steer_angle;
    double angular_velocity;
    double slip_angle;
    bool st_dyn;
};

}
