#include <rsp_frontier_exploration_server/front_exp_server.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "front_exp_server");
  ros::NodeHandle nh;
  Frontier_exploration fe;
  ros::ServiceServer ss = nh.advertiseService("get_frontier_exploration_goal", &Frontier_exploration::getGoal, &fe);
  ROS_INFO("Ready to get the current frontier exploration goal");
  ros::spin();
  return 0;
}