#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base/move_base.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "nav_msgs/OccupancyGrid.h"
#include "rsp_turtlebot3_msgs/RandomWalkAction.h"
#include "rsp_turtlebot3_msgs/rsp_turtlebot3_pose.h"
#include "rtt/TaskContext.hpp"
#include "sensor_msgs/LaserScan.h"

typedef actionlib::SimpleActionClient<rsp_turtlebot3_msgs::RandomWalkAction>
    RandomWalkActionClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseActionClient;

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

    void randomWalkClbk(double running_time);
    void stopClbk();

    // Subscriber callbacks
    void scanSubClbk(const sensor_msgs::LaserScanConstPtr& msg);
    void mapSubClbk(const nav_msgs::OccupancyGridConstPtr& msg);

   private:
    // get robot pose from tf
    rsp_turtlebot3_msgs::rsp_turtlebot3_pose getRobotPose(
        ros::Time stamp = ros::Time::now()) const;

    // move robot by sending action request to move_base
    void moveRobot(const rsp_turtlebot3_msgs::rsp_turtlebot3_pose& robot_pose);

    // move base action client callbacks
    void moveBaseDoneClbk(
        const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& result) const;

    void moveBaseActiveClbk() const;

    void moveBaseFeedbackClbk(
        const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) const;

    // random walk for certain amount of time[second]
    void randomWalk(double time);

    // random walk action callbacks
    void randomWalkDoneClbk(
        const actionlib::SimpleClientGoalState& state,
    const rsp_turtlebot3_msgs::RandomWalkResultConstPtr& result);

    void randomWalkActiveClbk() const;

    void randomWalkFeedbackClbk(
        const rsp_turtlebot3_msgs::RandomWalkFeedbackConstPtr& feedback) const;

    // tf listener
    tf::TransformListener tf_listener_;

    // node handle
    ros::NodeHandle nh_;

    // Wavefront frontier_exploration client
    ros::ServiceClient frontier_exlore_client_;

    // randomwalk action clinet
    std::unique_ptr<RandomWalkActionClient>

        random_walk_client_;

    // move base action client
    std::unique_ptr<MoveBaseActionClient> move_base_client_;

    // loop detection clients
    ros::ServiceClient add_scan_client_;
    ros::ServiceClient loop_detection_client_;

    // scan subscriber
    ros::Subscriber scan_sub_;

    // map subscriber
    ros::Subscriber map_sub_;

    bool detect_loop_;
    bool explore_;
    bool random_walk_;
    double random_walk_time_;

    // start time of the last goal send to move base actino client
    ros::Time last_goal_start_time_;

    sensor_msgs::LaserScan curr_scan_;
    nav_msgs::OccupancyGrid curr_map_;

    // cancel last goal sent to move base client after wait time [second]
    double move_base_goal_wait_time_;

    // duration for random walk when auto-exploration is stuck
    double random_walk_time_when_stuck_;

    // frame ids for robot pose tf look up
    std::string world_frame_;
    std::string robot_frame_;

    // RTT properties
    std::string scan_topic_;
    std::string map_topic_;

    std::string add_scan_service_name_;
    std::string detect_loop_service_name_;

    std::string get_exp_goal_service_name_;

    std::string random_walk_action_name_;
    std::string move_base_action_name_;
};