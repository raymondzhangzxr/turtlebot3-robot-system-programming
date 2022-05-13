#include <ros/ros.h>

#include "rsp_random_walk_exploration/rsp_random_walk_exploration.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rsp_random_walk_action");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;
    RandomWalkAction RandomWalkAction(nh, pnh);
    ROS_INFO("Ready to get the command for random walk");
    ros::spin();
    return 0;
}