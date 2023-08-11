#include "f1tenth_simulator/pose_2d.hpp"
#include "f1tenth_simulator/scan_simulator_2d.hpp"
#include "f1tenth_simulator/distance_transform.hpp"
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace racecar_simulator;

ScanSimulator2D::ScanSimulator2D(
    int num_beams_, 
    double field_of_view_, 
    double scan_std_dev_,
    double scan_max_range_,
    double cube_width_,
    double ray_tracing_epsilon_,
    int theta_discretization) 
  : num_beams(num_beams_),
    field_of_view(field_of_view_),
    scan_std_dev(scan_std_dev_),
    scan_max_range(scan_max_range_),
    cube_width(cube_width_),
    ray_tracing_epsilon(ray_tracing_epsilon_),
    theta_discretization(theta_discretization) {
    // Initialize laser settings
    angle_increment = field_of_view / (num_beams - 1);

    // Initialize the output
    scan_output = std::vector<double>(num_beams);

    // Initialize the noise
    noise_generator = std::mt19937(std::random_device{}());
    noise_dist = std::normal_distribution<double>(0., scan_std_dev);

    // It is very IMPORTANT to understand theta_index_increment and why they implemented this
    // angle_increment/(2 * M_PI) means how many beams there are if we scan 360 degree with current angular_increment
    // we know that with current angular_increment, there are 1080+1 beams for 270 degree
    // in our case, there will be 1440+1 beams for 360 degree
    // theta_discretization/theta_index_increment = 1440
    // hence, theta_index_increment = theta_discretization/1440
    // due to we are going to slice 360 degree into theta_discretization(2000+1) parts when precomputing sines cosines arctanes,
    // we need to know how many beams for 360 degree
    // so that we could know how many increments on theta_discretization if one beam incremented, in our case, that is 1.3888889
    // Imagine there are two gearwheel, one with 2000+1 teeth, another with 1440+1 teeth, but their radius is same.
    // theta_index_increment means how many teeth incremented in larger gear when smaller gear incremented 1 tooth.
    theta_index_increment = theta_discretization * angle_increment / (2 * M_PI);

    // Precompute sines, cosines and arctanes
    sines = std::vector<double>(theta_discretization + 1);
    cosines = std::vector<double>(theta_discretization + 1);
    arctanes = std::vector<double>(theta_discretization + 1);

    // slice 2PI into theta_discretization(2000+1) parts, and calculate sin cos arctan
    for (int i = 0; i <= theta_discretization; i++) {
        // calculate theta on the discretization from 0 to 2Pi
        double theta = (2 * M_PI * i) / ((double) theta_discretization);

        // calculate sin and cos of that theta value
        // the reason we precompute all sin and cos is that we can only compute sin and cos for each theta just one time
        // although we can calculate theta of beam easily and then calculate the sin and cos, imagine there are 1081 beams in one scan,
        // and in every second there are hundreds scans happened, but a lot of theta actually is calculated over and over again.
        // hence, better have a copy of all sin and cos, and when we need it, just directly get it from vector.
        sines[i] = std::sin(theta);
        cosines[i] = std::cos(theta);

        // this is for calculating slope of beams, so that we will know whether a beam intersects with the opponent car
        // due to theta actually starts from X axis which is y/x, so we need to calculate tan(theta) first and divided by 1, which will become x/y.
        // and when theta = 0 or Pi or 2Pi, tan(theta) will be 0, so avoid those number, although no exception will occur if not do this
        if (theta != 0 or theta != M_PI or theta != 2 * M_PI) {
            arctanes[i] = 1 / std::tan(theta);
        }
    }

    // this threshold is for reporting can see the opponent when the opponent is in the threshold distance
    threshold = 5;
}

const std::vector<double> ScanSimulator2D::scan(const Pose2D & pose, const Pose2D & opponent_pose, bool flag) {

    // this flag indicates that whether the car, called scan() method, want to know can see opponent or not
    // for example, blue car is calling scan method, and blue car don't want to know whether it can see red car or not,
    // then, blue car can pass false.
    flag_can_see_opponent = flag;
    can_see_opponent = false;

    scan(pose, opponent_pose, scan_output.data());
    return scan_output;
}

void ScanSimulator2D::scan(const Pose2D & pose, const Pose2D & opponent_pose, double * scan_data) {
    // Make theta discrete by mapping the range [-pi,pi] onto [0, theta_discretization)
    // field_of_view/2 = 3/4 PI
    // (pose.theta - field_of_view/2.)/(2 * M_PI) is to calculate the orientation of lidar scan starting beam
    // and after it times theta_discretization, the meaning becomes the start index in theta_discretization
    // Here, theta_index can be out of range [0, 2000], due to the pose.theta can be any radians rather than [-pi, pi]
    // this is the reason why we need the following steps to round theta_index into [0, 2000)
    double theta_index = theta_discretization * (pose.theta - field_of_view / 2.) / (2 * M_PI);

    // it is wrapped into (-theta_discretization, theta_discretization) first
    // fmod: mod% for float numbers, sign depends on the first number
    theta_index = std::fmod(theta_index, theta_discretization);
    // if theta_index less than 0, add a theta_discretization, then it will be greater than 0
    if (theta_index < 0) theta_index += theta_discretization;

    // process through each beam (1081).
    // For theta_index, we will only use number of theta_index that equal to number of beams
    for (int i = 0; i < num_beams; i++) {
        // Compute the distance to the nearest point
        scan_data[i] = trace_ray(pose.x, pose.y, theta_index, opponent_pose.x, opponent_pose.y, opponent_pose.theta);

        // Add Gaussian noise to the trace ray
        if (scan_std_dev > 0)
            scan_data[i] += noise_dist(noise_generator);

        // Increment the scan
        theta_index += theta_index_increment;
        // Make sure it stays in the range [0, theta_discretization)
        // although theta_index may out of range after added theta_index_increment,
        // this will make sure after it reaches boundary, it will start from 0 again
        while (theta_index >= theta_discretization)
            theta_index -= theta_discretization;
    }
}

double ScanSimulator2D::trace_ray(double x, double y, double theta_index, double opponent_X, double opponent_Y, double opponent_theta) {
    // Add 0.5 to make this operation round to ceil rather than floor
    // why?
    int theta_index_ = theta_index + 0.5;

    double s = sines[theta_index_];
    double c = cosines[theta_index_];
    // slope
    double k = arctanes[theta_index_];

    double original_x = x;
    double original_y = y;

    // Initialize the distance to the nearest obstacle
    // if x and y is out of racetrack, distance_to_nearest = 0, and while loop will be skipped
    double distance_to_nearest = distance_transform(x, y);
    double total_distance = distance_to_nearest;

    // when distance_to_nearest == 0, means either the car out of the map, or this beam reached the boundary of the racetrack
    while (distance_to_nearest != 0) {
        // Move in the direction of the ray
        // REMEMBER, pose.theta 0 is the positive direction of X axis not Y axis, and ALL theta related value starts from X axis not Y axis
        // hence, the idea is we know that the distance to nearest obstacle of current car position,
        // in order to get the distance to obstacle in a certain beam, we want to move 'distance to nearest obstacle' to that beam direction,
        // because we just move a distance to nearest obstacle, so we can make sure that this move will not overlap with an obstacle.
        // then, repeat the step, keep moving in the direction of that beam until reach an obstacle.
        // the result is sum of all the distance to nearest obstacle along the way.
        // And the movement can be divided into x direction and y direction,
        // due to theta value is angle between current beam and x direction, so x need to time cos, and y need to time sin.
        x += distance_to_nearest * c;
        y += distance_to_nearest * s;

        // get the nearest distance at that point
        distance_to_nearest = distance_transform(x, y);
        total_distance += distance_to_nearest;

        // you can comment off the if section below, and you will see something like Moiré pattern
        // when the racetrack is straight
        // this is because the size of grid in distance_transform is not small enough
        // We can apply a small distance to x and y, let them go back a little distance,
        // and try again to see whether the x and y still in the obstacle or not
        // this is the first time x and y out of the racetrack
        if (distance_to_nearest == 0) {
            double error = 0;
            while (distance_to_nearest == 0) {
                // accumulating error
                error += 0.01;
                // move back a little
                x -= 0.01 * c;
                y -= 0.01 * s;
                distance_to_nearest = distance_transform(x, y);
            }
            // minus total error
            total_distance -= error;
            break;
        }
    }

    // start calculating vehicle obstacle in LiDAR

    // bias of the beam x = ky + b
    double b = x - k * y;

    // calculate coordinate of four points of opponent car first,
    // x1 y1 is the first corner in anti-clockwise direction from opponent_theta
    // x2 y2 is the second corner, and so on
    double center_to_corner = sqrt(2) * cube_width / 2;

    double x1 = center_to_corner * std::cos(    M_PI / 4 + opponent_theta) + opponent_X;
    double y1 = center_to_corner * std::sin(    M_PI / 4 + opponent_theta) + opponent_Y;

    double x2 = center_to_corner * std::cos(3 * M_PI / 4 + opponent_theta) + opponent_X;
    double y2 = center_to_corner * std::sin(3 * M_PI / 4 + opponent_theta) + opponent_Y;

    double x3 = center_to_corner * std::cos(5 * M_PI / 4 + opponent_theta) + opponent_X;
    double y3 = center_to_corner * std::sin(5 * M_PI / 4 + opponent_theta) + opponent_Y;

    double x4 = center_to_corner * std::cos(7 * M_PI / 4 + opponent_theta) + opponent_X;
    double y4 = center_to_corner * std::sin(7 * M_PI / 4 + opponent_theta) + opponent_Y;

    std::vector<std::vector<double>> points = {{y1, y2, x1, x2},
                                               {y2, y3, x2, x3},
                                               {y3, y4, x3, x4},
                                               {y4, y1, x4, x1}};

    // decided whether this beam (line) intersects with opponent car (square) or not
    // if we put y coordinate of four corner points into the beam equation, and compare the result with actual x coordinate
    // if ALL the results greater or smaller than the actual x coordinate, which means there is no intersection between the beam and the square
    // if beam is a vertical line, which means slope is INF, we just compare y coordinate of each point to the scan y coordinate.
    if (theta_index_ == 0 or theta_index_ == theta_discretization / 2 or theta_index_ == theta_discretization) {
        if (((y1 > original_y) and (y2 > original_y) and (y3 > original_y) and (y4 > original_y)) or
            ((y1 < original_y) and (y2 < original_y) and (y3 < original_y) and (y4 < original_y))) {
            return std::min(total_distance, scan_max_range);
        }
    } else {
        if (((k * y1 + b > x1) and (k * y2 + b > x2) and (k * y3 + b > x3) and (k * y4 + b > x4)) or
            ((k * y1 + b < x1) and (k * y2 + b < x2) and (k * y3 + b < x3) and (k * y4 + b < x4))) {
            return std::min(total_distance, scan_max_range);
        }
    }

    // if the beam intersects with the opponent car, we can start calculate the actual distance
    // the opponent car should be between this car and the obstacle
    // Imagine there are three dots on this LiDAR beam: this car, the opponent car, the obstacle.
    // Draw a circle, the radius is the distance between this car and the opponent car.
    // Draw another circle, the radius is the distance between the opponent car and the obstacle.
    // each of the circle has two intersect points with this LiDAR beam.
    // For both circles, their radius need to be smaller than distance between this car and the obstacle,
    // which is the total length of this LiDAR beam.
    // this makes sure that there are one common intersect point and this point is between this car and the obstacle
    // calculate the distance between the LiDAR sensor and opponent car
    double this_to_opponent = sqrt(pow((original_x - opponent_X), 2) + pow((original_y - opponent_Y), 2));

    // for this car, the opponent car need to be closer than the obstacle
    if (this_to_opponent < total_distance) {

        // calculate distance between the obstacle and opponent car
        double obstacle_to_opponent = sqrt(pow((x - opponent_X), 2) + pow((y - opponent_Y), 2));

        // for the obstacle, the opponent car need to be closer than this car
        if (obstacle_to_opponent < total_distance) {

            // if you don't use MPC and overtaking algorithm, this if section can be ignored
            if (flag_can_see_opponent and this_to_opponent < threshold) {
                can_see_opponent = true;
                flag_can_see_opponent = false;
            }

            // if slope is INF, simply return distance between this car to opponent car and minus half square width
            if (theta_index_ == 0 or theta_index_ == theta_discretization / 2 or theta_index_ == theta_discretization) {
                return std::min(this_to_opponent - cube_width / 2, scan_max_range);
            }

            // calculating intersection points of the beam and four lines of the square, beam equation is x = ky + b,
            // lines of square is (x - x1)/(x2 - x1) = (y - y1)/(y2 - y1). put beam equation into the lines and simplify it
            double intersection_point1_y = (y2 * x1 - y2 * b + y1 * b - y1 * x2) / (k * y2 - k * y1 - x2 + x1);
            double intersection_point1_x = k * intersection_point1_y + b;

            double intersection_point2_y = (y3 * x2 - y3 * b + y2 * b - y2 * x3) / (k * y3 - k * y2 - x3 + x2);
            double intersection_point2_x = k * intersection_point2_y + b;

            double intersection_point3_y = (y4 * x3 - y4 * b + y3 * b - y3 * x4) / (k * y4 - k * y3 - x4 + x3);
            double intersection_point3_x = k * intersection_point3_y + b;

            double intersection_point4_y = (y1 * x4 - y1 * b + y4 * b - y4 * x1) / (k * y1 - k * y4 - x1 + x4);
            double intersection_point4_x = k * intersection_point4_y + b;

            std::vector<std::vector<double>> array = {{intersection_point1_y, intersection_point1_x},
                                                      {intersection_point2_y, intersection_point2_x},
                                                      {intersection_point3_y, intersection_point3_x},
                                                      {intersection_point4_y, intersection_point4_x}};
            double scan_to_square = scan_max_range;
            for (int i = 0; i < 4; i++) {
                // There will be four intersection point, but the real point we want is just one
                // the real point is the one within the cube range AND the closest one
                // first of all, filter all points that not in square, which means both y and x is not between two points
                // filter y
                if ((points[i][0] + ray_tracing_epsilon >= array[i][0] and
                     array[i][0] >= points[i][1] - ray_tracing_epsilon) or
                    (points[i][0] - ray_tracing_epsilon <= array[i][0] and
                     array[i][0] <= points[i][1] + ray_tracing_epsilon)) {
                    // filter x
                    if ((points[i][2] + ray_tracing_epsilon >= array[i][1] and
                         array[i][1] >= points[i][3] - ray_tracing_epsilon) or
                        (points[i][2] - ray_tracing_epsilon <= array[i][1] and
                         array[i][1] <= points[i][3] + ray_tracing_epsilon)) {
                        // only one or two points left, we just pick out the smallest one, which is the first intersect points
                        scan_to_square = std::min(scan_to_square, sqrt(pow((original_x - array[i][1]), 2) +
                                                                       pow((original_y - array[i][0]), 2)));
                    }
                }
            }
            return scan_to_square;
        }
    }
    return std::min(total_distance, scan_max_range);
}

double ScanSimulator2D::distance_transform(double x, double y) const {
  // Convert the pose to a grid cell
  int cell = xy_to_cell(x, y);
  if (cell < 0) return 0;
  return dt[cell];
}

int ScanSimulator2D::xy_to_cell(double x, double y) const {
    int row, col;
    xy_to_row_col(x, y, &row, &col);
    return row_col_to_cell(row, col);
}

void ScanSimulator2D::xy_to_row_col(double x, double y, int * row, int * col) const {
    // distance between x or y and the bottom left conner of the original map image
    double x_trans = x - origin.x;
    double y_trans = y - origin.y;

    // Rotate the x and y into the map frame of reference
    // usually the theta of map is 0, hence, sin = 0 and cos = 1
    // imagine the map is a rectangle, and the car is located in the map.
    // there are two coordinate of the car, one is about the ground truth frame of reference and another is about the map reference
    // x and y is the coordinate about the ground truth, but we want coordinate of the map.
    // better calculate these on a paper, just some high school math.
    double x_rot =   x_trans * origin_c + y_trans * origin_s;
    double y_rot = - x_trans * origin_s + y_trans * origin_c;

    // if the car is located outside the map
    if (x_rot < 0 or x_rot >= width * resolution or y_rot < 0 or y_rot >= height * resolution) {
        *col = -1;
        *row = -1;
    } else {
        // transform coordinate of the car about the map into width and height
        // in Rviz, x positive is up, y positive is left
        // x direction is width, y direction is height
        // col is width, row is height
        *col = std::floor(x_rot/resolution);
        *row = std::floor(y_rot/resolution);
    }
}

int ScanSimulator2D::row_col_to_cell(int row, int col) const {
    // calculate how many full rows first, then plus the last row
    return row * width + col;
}

bool ScanSimulator2D::see_opponent(){
    return can_see_opponent;
}
// overload for changing map on the fly
void ScanSimulator2D::set_map(const std::vector<double> & map, double free_threshold) {
    for (size_t i = 0; i < map.size(); i++) {
        if (0 <= map[i] and map[i] <= free_threshold) {
            dt[i] = 99999; // Free
        } else {
            dt[i] = 0; // Occupied
        }
    }
    DistanceTransform::distance_2d(dt, width, height, resolution);
}

void ScanSimulator2D::set_map(
        const std::vector<double> & map,
        size_t height_,
        size_t width_,
        double resolution_,
        const Pose2D & origin_,
        double free_threshold) {

    // Assign parameters
    height = height_;
    width = width_;
    resolution = resolution_;
    origin = origin_;
    origin_c = std::cos(origin.theta);
    origin_s = std::sin(origin.theta);

    // Threshold the map
    dt = std::vector<double>(map.size());
    for (size_t i = 0; i < map.size(); i++) {
        if (0 <= map[i] and map[i] <= free_threshold) {
            dt[i] = 99999; // Free
        } else {
            dt[i] = 0; // Occupied
        }
    }
    // this is to calculate the distance from each pixel to the nearest occupied pixel in coordinate
    // so the elements in dt vector represent the distance and we can directly use them.
    DistanceTransform::distance_2d(dt, width, height, resolution);
}
