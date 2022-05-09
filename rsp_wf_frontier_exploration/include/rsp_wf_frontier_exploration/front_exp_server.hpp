#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "rsp_turtlebot3_msgs/get_frontier_exploration_goal.h"
typedef rsp_turtlebot3_msgs::rsp_turtlebot3_pose turtle_pose;

class FrontierExplorer {
   public:
    FrontierExplorer(ros::NodeHandle& nh);
    bool getGoal(
        rsp_turtlebot3_msgs::get_frontier_exploration_goal::Request& req,
        rsp_turtlebot3_msgs::get_frontier_exploration_goal::Response& res);

   private:
    ros::NodeHandle _nh;
    ros::ServiceServer _server;
};