#include <rsp_wf_frontier_exploration/front_exp_server.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "front_exp_server");
    ros::NodeHandle nh;
    FrontierExplorer fe(nh);
    ROS_INFO("Ready to get the current frontier exploration goal");
    ros::spin();
    return 0;
}