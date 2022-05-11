#include <ros/ros.h>
#include <rsp_turtlebot3_msgs/RandomWalkAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h> 
#include <algorithm>


class randomWalk_action{
    private:
        // Use typedef to simplified variable name in cpp
        
        typedef actionlib::SimpleActionServer<rsp_turtlebot3_msgs::RandomWalkAction> RandomWalkServer;
        ros::NodeHandle nh;
        // SimpleActionServer a template in actionlib class
        //  Initialize it with the action we just created
        // Pointer points to a memory that does not have any entity yet, in cpp reset it
        std::unique_ptr<RandomWalkServer> as;
        // as: action server
    
    public:
        randomWalk_action(ros::NodeHandle& nh);
        ~randomWalk_action();
        void executeCB(const rsp_turtlebot3_msgs::RandomWalkGoalConstPtr &goal);
};
