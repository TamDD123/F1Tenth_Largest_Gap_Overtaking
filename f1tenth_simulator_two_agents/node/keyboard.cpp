#include <ros/ros.h>

#include <std_msgs/String.h>

#include <termios.h>

#include <stdio.h>
#include <signal.h>

// for printing
#include <iostream>

// https://stackoverflow.com/questions/4437527/why-do-we-use-volatile-keyword
// https://stackoverflow.com/questions/24931456/how-does-sig-atomic-t-actually-work
static volatile sig_atomic_t keep_running = 1;


void sigHandler(int not_used) {
    ROS_INFO_STREAM("Interrupt signal (" << not_used << ") received.");
    // 0 == false
    keep_running = 0;
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "keyboard");
    // Initialize Node Handle
    ros::NodeHandle n = ros::NodeHandle("~");

    // Initialze publisher
    std::string keyboard_topic;
    n.getParam("keyboard_topic", keyboard_topic);
    ros::Publisher key_pub = n.advertise<std_msgs::String>(keyboard_topic, 10);

    //  struct termios {
    //      tcflag_t c_iflag;		/* input mode flags */
    //      tcflag_t c_oflag;		/* output mode flags */
    //      tcflag_t c_cflag;		/* control mode flags */
    //      tcflag_t c_lflag;		/* local mode flags */
    //      cc_t c_line;			/* line discipline */
    //      cc_t c_cc[NCCS];		/* control characters */
    //      speed_t c_ispeed;		/* input speed */
    //      speed_t c_ospeed;		/* output speed */
    //  };
    // https://www.ibm.com/docs/en/zos/2.2.0?topic=functions-tcsetattr-set-attributes-terminal

    static struct termios old_setting, new_setting;

    // store the original setting of a terminal
    tcgetattr(STDIN_FILENO, &old_setting);

    // make a new setting of terminal
    new_setting = old_setting;

    // c_lflag is the bitwise inclusive-OR of a number of kinds of symbols
    // ICANON is called line mode. Input is not delivered to the application until an entire line has been input.
    // ICANON is the second bit in bitwise, if it is on, that bit should be 1
    // ICANON = 0010
    // ~ICANON = 1101
    // &: only 1 when both is 1, otherwise 0
    // & operation doesn't affect other bit, because all the bit is 1 except the second in ~ICANON,
    // So, if the original is 1, then after & operation it is still 1
    // so that ICANON bit always 0, which means ICANON is turned off, the system does not wait for the user to enter a complete line.
    new_setting.c_lflag &= ~(ICANON);

    // set the new setting to the terminal
    // TCSANOW means the change should take place immediately.
    tcsetattr(STDIN_FILENO, TCSANOW, &new_setting);

    /*
        struct sigaction {
            void       (*sa_handler)(int);
            sigset_t   sa_mask;
            int        sa_flags;
            void       (*sa_sigaction)(int, siginfo_t *, void *);
        };
     https://www.ibm.com/docs/en/zos/2.2.0?topic=functions-sigaction-examine-change-signal-action
    */
    struct sigaction act;
    // the action we are going to do
    act.sa_handler = sigHandler;
    // SIGINT is when the thread heard this certain signal, and takes some actions.
    // SIGINT means Ctrl+C which is shut down the application in a terminal
    // so, after we received Ctrl+C, we want to stop listening to keyboard
    // act contains the actions after receive the signal
    // NULL pointer, in which case sigaction() does not store this information.
    sigaction(SIGINT, &act, NULL);

    std_msgs::String msg;
    int c;
    // keep_running will be changed by sigHandler after Ctrl+C happened
    while ((ros::ok()) && (keep_running)) {
        // get the character pressed, blocking when not hearing anything
        c = getchar();

        // Publish the character 
        msg.data = c;
        key_pub.publish(msg);
    }
    // set back terminal setting
    tcsetattr(STDIN_FILENO, TCSANOW, &old_setting);
    
    return 0;
}