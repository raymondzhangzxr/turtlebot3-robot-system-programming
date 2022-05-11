#include<ros/ros.h>
#include <rsp_random_walk_frontier_exploration/randomWalk_action.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "rsp_random_walk_action_node");//initialize the node
    ros::NodeHandle nh;
    randomWalk_action randomWalk_action(nh);
    ROS_INFO("Ready to get the command for random walk");

    ros::spin();

    return 0;
}