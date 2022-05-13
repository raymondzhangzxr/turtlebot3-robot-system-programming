#include <algorithm>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "rsp_turtlebot3_msgs/RandomWalkAction.h"
#include "sensor_msgs/LaserScan.h"

typedef actionlib::SimpleActionServer<rsp_turtlebot3_msgs::RandomWalkAction>
    RandomWalkServer;

class RandomWalkAction {
   public:
    RandomWalkAction(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~RandomWalkAction();
    // path generator
    void linearPathGenerator() const;
    // operation callback
    void executeClbk(const rsp_turtlebot3_msgs::RandomWalkGoalConstPtr& goal); 
    // Subscriber callback, and tell where is collision
    void scanSubClbk(const sensor_msgs::LaserScanConstPtr& msg);
    void pubZeroTwist() const;

   private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    // as: action server
    std::unique_ptr<RandomWalkServer> as_;
    // current scan
    sensor_msgs::LaserScan curr_scan_;
    ros::Publisher vel_pub_;
    ros::Subscriber scan_sub_;
    enum class CollisionType {
        NoCollision = 0,
        FrontCollsion = 1,
        RearCollision = 2,
    };
    CollisionType collision_region_;

    float linear_speed_;
    float angular_speed_;
    float threshold_;
    float bubble_size_;
    float CCW_prob_;
    int rate_;
};
