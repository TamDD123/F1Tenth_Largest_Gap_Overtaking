#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>

#include <fstream>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"

using namespace racecar_simulator;

/**
 * this node is for switching between different controller
 */
class BehaviorController {
private:
    // A ROS node
    ros::NodeHandle n;

    //ros::Subscriber joy_sub;
    ros::Subscriber key_sub;
    ros::Subscriber brake_bool_sub;
    ros::Subscriber switch_sub_red;

    // Publisher for mux controller
    ros::Publisher mux_pub;
    // Publisher for starting logging data signal
    ros::Publisher data_pub;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size;

    // Mux indices
    //int joy_mux_idx;
    int key_mux_idx;
    int random_walker_mux_idx;
    int nav_mux_idx;
    int brake_mux_idx;
    int pp_mux_idx;
    int pp2_mux_idx;
    // ***Add mux index for new planner here***
    // int new_mux_idx;

    // Button indices
    //int joy_button_idx;
    int key_button_idx;
    int random_walk_button_idx;
    int brake_button_idx;
    int nav_button_idx;
    // ***Add button index for new planner here***
    // int new_button_idx;

    // Key indices
    //std::string joy_key_char;
    std::string keyboard_key_char;
    std::string brake_key_char;
    std::string random_walk_key_char;
    std::string nav_key_char;
    std::string pp_key_char;
    std::string pp2_key_char;
    std::string data_key_char;
    // ***Add key char for new planner here***
    // int new_key_char;

    // Is brake on? (not engaged, but on)
    bool safety_on;

    // flag
    bool log_data = false;

    // {0: joy, 1: key, 2: random_walker, 3: brake, 4: nav, 5: MPC, 6: overtaking, 7: null, 8: null, 9: null, 10: null}
    //std::vector<bool> joy_button_previous {false, false, false, false, false, false, false, false, false, false, false};

public:
    BehaviorController() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string //joy_topic, 
keyboard_topic, brake_bool_topic, mux_topic, data_topic, switch_topic_red;
        //n.getParam("joy_topic", joy_topic);
        n.getParam("mux_topic", mux_topic);
        n.getParam("keyboard_topic", keyboard_topic);
        n.getParam("brake_bool_topic", brake_bool_topic);

        // Make a publisher for mux messages
        mux_pub = n.advertise<std_msgs::Int32MultiArray>(mux_topic, 10);
        // publisher for sending signal
        data_pub = n.advertise<std_msgs::Bool>(data_topic, 1);

        //joy_sub = n.subscribe(joy_topic, 1, &BehaviorController::joy_callback, this);
        key_sub = n.subscribe(keyboard_topic, 1, &BehaviorController::key_callback, this);
        brake_bool_sub = n.subscribe(brake_bool_topic, 1, &BehaviorController::brake_callback, this);
        switch_sub_red = n.subscribe(switch_topic_red, 1, &BehaviorController::switch_callback, this);

        // Get mux indices
        //n.getParam("joy_mux_idx", joy_mux_idx);
        n.getParam("key_mux_idx", key_mux_idx);
        n.getParam("random_walker_mux_idx", random_walker_mux_idx);
        n.getParam("brake_mux_idx", brake_mux_idx);
        n.getParam("nav_mux_idx", nav_mux_idx);
        n.getParam("pp_mux_idx", pp_mux_idx);
	n.getParam("pp2_mux_idx", pp2_mux_idx);
        // ***Add mux index for new planner here***
        // n.getParam("new_mux_idx", new_mux_idx);

        // Get joy indices
        //n.getParam("joy_button_idx", joy_button_idx);
        n.getParam("key_button_idx", key_button_idx);
        n.getParam("random_walk_button_idx", random_walk_button_idx);
        n.getParam("brake_button_idx", brake_button_idx);
        n.getParam("nav_button_idx", nav_button_idx);
        // ***Add button index for new planner here***
        // n.getParam("new_button_idx", new_button_idx);

        // Get key indices
        //n.getParam("joy_key_char", joy_key_char);
        n.getParam("keyboard_key_char", keyboard_key_char);
        n.getParam("random_walk_key_char", random_walk_key_char);
        n.getParam("brake_key_char", brake_key_char);
        n.getParam("nav_key_char", nav_key_char);
        n.getParam("pp_key_char", pp_key_char);
	n.getParam("pp2_key_char", pp2_key_char);
	n.getParam("data_key_char", pp_key_char);
        // ***Add key char for new planner here***
        // n.getParam("new_key_char", new_key_char);

        // Initialize the mux controller 
        n.getParam("mux_size", mux_size);
        mux_controller.reserve(mux_size);
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }

        // Start with ebrake off
        safety_on = false;

    }

    /// ---------------------- GENERAL HELPER FUNCTIONS ----------------------

    void publish_mux() {
        // make mux message
        std_msgs::Int32MultiArray mux_msg;
        mux_msg.data.clear();
        // push data onto message
        for (int i = 0; i < mux_size; i++) {
            mux_msg.data.push_back(int(mux_controller[i]));
        }

        // publish mux message
        mux_pub.publish(mux_msg);
    }

    void toggle_mux(int mux_idx, std::string driver_name) {
        // This takes in an index and the name of the planner/driver and 
        // toggles the mux appropiately
        if (mux_controller[mux_idx]) {
            ROS_INFO_STREAM(driver_name << " turned off");
            mux_controller[mux_idx] = false;
            publish_mux();
        }
        else {
            ROS_INFO_STREAM(driver_name << " turned on");
            mux_controller[mux_idx] = true;
            publish_mux();
        }
    }

    void toggle_brake_mux() {
        ROS_INFO_STREAM("Emergency brake engaged");
        // turn everything off
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }
        // turn on desired controller
        mux_controller[brake_mux_idx] = true;

        publish_mux();
    }

// ---------------------- CALLBACK FUNCTIONS ---------------------------------------------------------------------------

    void brake_callback(const std_msgs::Bool & msg) {
        if (msg.data && safety_on) {
            toggle_brake_mux();
        } else if (!msg.data && mux_controller[brake_mux_idx]) {
            mux_controller[brake_mux_idx] = false;
        }
    }

    /*void joy_callback(const sensor_msgs::Joy & msg) {
        // the reason joy_callback looks different from key_callback is due to the different way of message publishing
        // In keyboard node, when a key is press down, a message is published, like a switch
        // In joy_node, if you move a joystick, the node will keep publishing data, like a data stream
        // So, when a button is down, with a joystick is moving, the message will contain same data for the button,
        // e.g. a lot of on
        // but this will lead to start and stop recording data over and over if we use implementation like keyboard
        // what we really want is, only change the flag when the button is down for the first time rather than holding it

        // if the button is on in the msg
        

        if (msg.buttons[joy_button_idx]) {
            if (!joy_button_previous[joy_button_idx]) {
                joy_button_previous[joy_button_idx] = true;
                // joystick
                toggle_mux(joy_mux_idx, "Joystick");
            }
        } else {
            joy_button_previous[joy_button_idx] = false;
        }

        if (msg.buttons[key_button_idx]) {
            if (!joy_button_previous[key_button_idx]) {
                joy_button_previous[key_button_idx] = true;
                // keyboard
                toggle_mux(key_mux_idx, "Keyboard");
            }
        } else {
            joy_button_previous[key_button_idx] = false;
        }

        if (msg.buttons[brake_button_idx]) {
            if (!joy_button_previous[brake_button_idx]) {
                joy_button_previous[brake_button_idx] = true;
                // emergency brake
                if (safety_on) {
                    ROS_INFO("Emergency brake turned off");
                    safety_on = false;
                } else {
                    ROS_INFO("Emergency brake turned on");
                    safety_on = true;
                }
            }
        } else {
            joy_button_previous[brake_button_idx] = false;
        }

        if (msg.buttons[random_walk_button_idx]) {
            if (!joy_button_previous[random_walk_button_idx]) {
                joy_button_previous[random_walk_button_idx] = true;
                // random walker
                toggle_mux(random_walker_mux_idx, "Random Walker");
            }
        } else {
            joy_button_previous[random_walk_button_idx] = false;
        }

        if (msg.buttons[nav_button_idx]) {
            if (!joy_button_previous[nav_button_idx]) {
                joy_button_previous[nav_button_idx] = true;
                // nav
                toggle_mux(nav_mux_idx, "Navigation");
            }
        } else {
            joy_button_previous[nav_button_idx] = false;
        }

//         ***Add new else if statement here for new planning method***
//         if (msg.buttons/axes[new_button_idx]) {
//             if (!joy_button_previous[new_button_idx]) {
//                 joy_button_previous[new_button_idx] = true;
//                 // new planner
//                 toggle_mux(new_mux_idx, "New Planner");
//             }
//         } else {
//             joy_button_previous[new_button_idx] = false;
//         }

    };*/

    void key_callback(const std_msgs::String & msg) {

        // Changing mux controller:
        //if (msg.data == joy_key_char) {
            // joystick
            //toggle_mux(joy_mux_idx, "Joystick");
        //} else
 	if (msg.data == keyboard_key_char) {
            // keyboard
            toggle_mux(key_mux_idx, "Keyboard");
        } else if (msg.data == brake_key_char) {
            // emergency brake 
            if (safety_on) {
                ROS_INFO("Emergency brake turned off");
                safety_on = false;
            } else {
                ROS_INFO("Emergency brake turned on");
                safety_on = true;
            }
        } else if (msg.data == random_walk_key_char) {
            // random walker
            toggle_mux(random_walker_mux_idx, "Random Walker");
        } else if (msg.data == nav_key_char) {
            // nav
            toggle_mux(nav_mux_idx, "Navigation");
        } else if (msg.data == pp_key_char) {
            // pp
            toggle_mux(pp_mux_idx, "Pure pursuit");
	    log_data = !log_data;
	    std_msgs::Bool msg;
	    msg.data = log_data;
	    //std::cout <<log_data <<std::endl;
	    data_pub.publish(msg);
	    //std::cout << data_pub << std::endl;
        } else if (msg.data == pp2_key_char) {
	    // pp2
	    toggle_mux(pp2_mux_idx, "Pure pursuit2");
	};

        // ***Add new else if statement here for new planning method***
        // else if (msg.data == new_key_char) {
        //  // new planner
        //  toggle_mux(new_mux_idx, "New Planner");
        // }

    }

    void switch_callback(const std_msgs::Bool & msg) {
//        if (msg.data and mux_controller[mpc_mux_idx]) {
//            toggle_mux(mpc_mux_idx, "Model Predictive Control");
//            toggle_mux(lstm_mux_idx, "LSTM overtaking");
//        } else if (!msg.data and mux_controller[lstm_mux_idx]) {
//            toggle_mux(lstm_mux_idx, "LSTM overtaking");
//            toggle_mux(mpc_mux_idx, "Model Predictive Control");
//        }
    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "behavior_controller");
    BehaviorController bc;
    ros::spin();
    return 0;
}
