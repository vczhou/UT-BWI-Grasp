#include <ros/ros.h>

#include <signal.h>

//action for grasping
#include "bwi_grasp/GpdGraspAction.h"

#include <segbot_arm_manipulation/arm_utils.h>

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
    g_caught_sigint = true;
    ROS_INFO("caught sigint, init shutdown sequence...");
    ros::shutdown();
    exit(1);
};

// Blocking call for user input
void pressEnter(std::string message) {
    std::cout << message;
    while (true){
        char c = std::cin.get();
        if (c == '\n')
            break;
        else if (c == 'q'){
            ros::shutdown();
            exit(1);
        }
        else {
            std::cout <<  message;
        }
    }
}

int main(int argc, char **argv) {
    // Intialize ROS with this node name
    ros::init(argc, argv, "demo_grasp_action_client");
    ros::NodeHandle n;
    
    //register ctrl-c
    signal(SIGINT, sig_handler);
    
    pressEnter("Demo starting.");
    
    actionlib::SimpleActionClient<bwi_grasp::GpdGraspAction> ac("gpd_grasp_as",true);
    ac.waitForServer();
    
    
    segbot_arm_manipulation::TabletopApproachGoal approach_goal;
    approach_goal.command = "grasp";
    
    //send the goal
    ROS_INFO("Sending goal to grasp object...");
    ac.sendGoal(approach_goal);
    
    //block until the action is completed
    ac.waitForResult();
}