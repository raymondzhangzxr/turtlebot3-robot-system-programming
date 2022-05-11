#include "ros/ros.h"
#include "rsp_turtlebot3_msgs/RandomWalkAction.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <algorithm>
#include <vector>


class RandomWalk{
    public:
        RandomWalk(ros::NodeHandle& nh);
        ~RandomWalk();
        // path generator
        void linearPathGenerator();
        // operation callback
        void executeCB(const rsp_turtlebot3_msgs::RandomWalkGoalConstPtr &goal);
        // Subscriber callback, and tell where is collision
        void scanSubCB(const sensor_msgs::LaserScanConstPtr& msg);
        void zeroTwist();
    
    private:
        typedef actionlib::SimpleActionServer<rsp_turtlebot3_msgs::RandomWalkAction> _RandomWalkServer;
        int _collision_region;
        ros::NodeHandle _nh;
        // as: action server
        std::unique_ptr<_RandomWalkServer> _as;
        // velocity control
        geometry_msgs::Twist _twist;
        // current scan
        sensor_msgs::LaserScan _curr_scan;
        ros::Publisher _vel_pub;
        ros::Subscriber _scan_sub;
        enum collisonConstants{
            _frontCollsion=1,
            _rearCollision=2,
            _noCollision=0,
        };
        float _linearSpeed;
        float _angularSpeed;
        float _threshold;
        float _bubbleSize;
        float _CCW_prob;
        int _rate;
};
