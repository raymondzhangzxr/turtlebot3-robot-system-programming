#include "ros/ros.h"
#include "rsp_turtlebot3_msgs/get_frontier_exploration_goal.h"
#include "nav_msgs/OccupancyGrid.h"
typedef rsp_turtlebot3_msgs::rsp_turtlebot3_pose turtle_pose;

 class Frontier_exploration
 {
  public:
   bool getGoal(rsp_turtlebot3_msgs::get_frontier_exploration_goal::Request& req,
              rsp_turtlebot3_msgs::get_frontier_exploration_goal::Response& res);

 };