#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base/move_base.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "rsp_turtlebot3_msgs/RandomWalkAction.h"
#include "nav_msgs/OccupancyGrid.h"
#include "rsp_turtlebot3_msgs/rsp_turtlebot3_pose.h"
#include "rtt/TaskContext.hpp"
#include "sensor_msgs/LaserScan.h"

class RttTurtlebot3 : public RTT::TaskContext {
   public:
    RttTurtlebot3(const std::string& name);
    ~RttTurtlebot3();

    bool configureHook() override;
    bool startHook() override;

    void updateHook() override;

    void stopHook() override;
    void cleanupHook() override;

    // RTT operation callbacks
    rsp_turtlebot3_msgs::rsp_turtlebot3_pose getRobotPoseClbk() const;
    void setRobotPosClbk(
        const rsp_turtlebot3_msgs::rsp_turtlebot3_pose& jnt_pos);
    void detectLoopClbk();
    void exploreClbk();

    void randomWalkClbk();
    void stopClbk();

    // Subscriber callbacks
    void scanSubClbk(const sensor_msgs::LaserScanConstPtr& msg);
    void mapSubClbk(const nav_msgs::OccupancyGridConstPtr& msg);

   private:

    // get robot pose from tf
    rsp_turtlebot3_msgs::rsp_turtlebot3_pose _get_robot_pose(
        ros::Time stamp = ros::Time::now()) const;

    // move robot by sending action request to move_base
    void _move_robot(
        const rsp_turtlebot3_msgs::rsp_turtlebot3_pose& robot_pose);

    // move base action client callbacks
    void _move_base_done_clbk(
        const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& result) const;

    void _move_base_active_clbk() const;

    void _move_base_feedback_clbk(
        const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) const;


    // move base action client
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
        _move_base_client;

    // tf listener
    tf::TransformListener _tf_listener;

    // node handle
    ros::NodeHandle _nh;

    // frontier_exploration client
    // Wavefront 
    ros::ServiceClient _frontier_exlore_client;
    // randomwalk action clinet
    actionlib::SimpleActionClient<rsp_turtlebot3_msgs::RandomWalkAction>
     _random_walk_client;

    // loop detection clients
    ros::ServiceClient _add_scan_client;
    ros::ServiceClient _loop_detection_client;

    // scan subscriber
    ros::Subscriber _scan_sub;

    // map subscriber
    ros::Subscriber _map_sub;

    bool _detect_loop;
    bool _explore;
    bool _randomWalk;
    
    ros::Time _last_goal_start_time;
    sensor_msgs::LaserScan _curr_scan;
    nav_msgs::OccupancyGrid _curr_map;
};