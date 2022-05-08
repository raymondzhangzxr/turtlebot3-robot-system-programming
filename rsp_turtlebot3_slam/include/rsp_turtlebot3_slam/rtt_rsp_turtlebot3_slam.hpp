#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base/move_base.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "rsp_turtlebot3_msgs/rsp_turtlebot3_pose.h"
#include "rtt/TaskContext.hpp"

class RttTurtlebot3 : public RTT::TaskContext {
   public:
    RttTurtlebot3(const std::string& name);
    ~RttTurtlebot3();

    bool configureHook() override;
    bool startHook() override;

    void updateHook() override;

    void stopHook() override;
    void cleanupHook() override;

    rsp_turtlebot3_msgs::rsp_turtlebot3_pose getRobotPose();
    void setRobotPos(const rsp_turtlebot3_msgs::rsp_turtlebot3_pose& jnt_pos);

   private:
    rsp_turtlebot3_msgs::rsp_turtlebot3_pose _get_robot_pose() const;

    void _move_base_done_clbk(
        const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& result) const;

    void _move_base_active_clbk() const;

    void _move_base_feedback_clbk(
        const  move_base_msgs::MoveBaseFeedbackConstPtr& feedback) const;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
        _move_base_client;

    tf::TransformListener _tf_listener;
};