#include "rsp_turtlebot3_slam/rtt_rsp_turtlebot3_slam.hpp"

#include <ros/ros.h>

#include "rtt/Component.hpp"
RttTurtlebot3::RttTurtlebot3(const std::string& name)
    : RTT::TaskContext(name), _move_base_client("move_base") {
    addOperation("GetRobotPose", &RttTurtlebot3::getRobotPose, this,
                 RTT::OwnThread);
    addOperation("SetRobotPose", &RttTurtlebot3::setRobotPos, this,
                 RTT::OwnThread);
    _move_base_client.waitForServer();
}

RttTurtlebot3::~RttTurtlebot3() {}

bool RttTurtlebot3::configureHook() { return true; }
bool RttTurtlebot3::startHook() { return true; }

void RttTurtlebot3::updateHook() {}

void RttTurtlebot3::stopHook() {}
void RttTurtlebot3::cleanupHook() {}

rsp_turtlebot3_msgs::rsp_turtlebot3_pose RttTurtlebot3::getRobotPose() {
    return _get_robot_pose();
}
void RttTurtlebot3::setRobotPos(
    const rsp_turtlebot3_msgs::rsp_turtlebot3_pose& robot_pose) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = robot_pose.x;
    goal.target_pose.pose.position.y = robot_pose.y;

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, robot_pose.yaw);

    goal.target_pose.pose.orientation.w = q.w();
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();

    _move_base_client.sendGoal(
        goal, boost::bind(&RttTurtlebot3::_move_base_done_clbk, this, _1, _2),
        boost::bind(&RttTurtlebot3::_move_base_active_clbk, this),
        boost::bind(&RttTurtlebot3::_move_base_feedback_clbk, this, _1));
    // _move_base_client.waitForResult(ros::Duration(10.0));
}

double get_yaw_from_quaternion(double x, double y, double z, double w) {
    tf::Quaternion tf_q(x, y, z, w);
    tf::Matrix3x3 rot_m(tf_q);
    double roll, pitch, yaw;
    rot_m.getRPY(roll, pitch, yaw);
    return yaw;
}

rsp_turtlebot3_msgs::rsp_turtlebot3_pose RttTurtlebot3::_get_robot_pose()
    const {
    tf::StampedTransform transform;
    rsp_turtlebot3_msgs::rsp_turtlebot3_pose pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.yaw = 0.0;
    try {
        _tf_listener.lookupTransform("/map", "/base_footprint", ros::Time(0),
                                     transform);
        pose.x = transform.getOrigin().x();
        pose.y = transform.getOrigin().y();
        const auto q = transform.getRotation();
        pose.yaw = get_yaw_from_quaternion(q.x(), q.y(), q.z(), q.w());
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
    return pose;
}

void RttTurtlebot3::_move_base_done_clbk(
    const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result) const {
    ROS_INFO_STREAM("Moved robot to goal position");
}

void RttTurtlebot3::_move_base_active_clbk() const {}

void RttTurtlebot3::_move_base_feedback_clbk(
    const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) const {
    const double x = feedback->base_position.pose.position.x;
    const double y = feedback->base_position.pose.position.y;
    const auto q = feedback->base_position.pose.orientation;
    const double yaw = get_yaw_from_quaternion(q.x, q.y, q.z, q.w);

    ROS_INFO_STREAM("Current robot pose [x, y, yaw] = " << x << " " << y << " "
                                                        << yaw);
}

ORO_CREATE_COMPONENT(RttTurtlebot3);