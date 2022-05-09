#include "rsp_turtlebot3_slam/rtt_rsp_turtlebot3_slam.hpp"

#include <ros/ros.h>

#include "rsp_turtlebot3_msgs/add_scan_to_loop_detection_database.h"
#include "rsp_turtlebot3_msgs/detect_loop.h"
#include "rsp_turtlebot3_msgs/get_frontier_exploration_goal.h"
#include "rtt/Component.hpp"

RttTurtlebot3::RttTurtlebot3(const std::string& name)
    : RTT::TaskContext(name),
      _move_base_client("move_base", true),
      _detect_loop(false),
      _explore(false) {
    // setup operations
    addOperation("GetRobotPose", &RttTurtlebot3::getRobotPoseClbk, this,
                 RTT::OwnThread);
    addOperation("SetRobotPose", &RttTurtlebot3::setRobotPosClbk, this,
                 RTT::OwnThread);
    addOperation("LoopDetection", &RttTurtlebot3::detectLoopClbk, this,
                 RTT::OwnThread);
    addOperation("Explore", &RttTurtlebot3::exploreClbk, this, RTT::OwnThread);

    _move_base_client.waitForServer();

    // loop detection clients
    std::string add_scan_service;
    std::string loop_detection_service;
    _nh.param<std::string>("/rsp_loop_detection/add_scan_service",
                           add_scan_service, "add_scan");
    _nh.param<std::string>("/rsp_loop_detection/detect_loop_service",
                           add_scan_service, "add_scan");

    _add_scan_client = _nh.serviceClient<
        rsp_turtlebot3_msgs::add_scan_to_loop_detection_database>(
        "/rsp_loop_detection/" + add_scan_service);
    _loop_detection_client =
        _nh.serviceClient<rsp_turtlebot3_msgs::detect_loop>(
            "/rsp_loop_detection/" + loop_detection_service);
    _add_scan_client.waitForExistence();
    _loop_detection_client.waitForExistence();

    // frontier exploration client
    _frontier_exlore_client =
        _nh.serviceClient<rsp_turtlebot3_msgs::get_frontier_exploration_goal>(
            "get_frontier_exploration_goal");
    _frontier_exlore_client.waitForExistence();

    // setup subscribers
    _scan_sub = _nh.subscribe("scan", 1000, &RttTurtlebot3::scanSubClbk, this);
    _map_sub = _nh.subscribe("map", 1000, &RttTurtlebot3::mapSubClbk, this);
}

RttTurtlebot3::~RttTurtlebot3() {}

/* #################### RTT hooks #################### */
bool RttTurtlebot3::configureHook() { return true; }
bool RttTurtlebot3::startHook() { return true; }

void RttTurtlebot3::updateHook() {
    if (!_detect_loop) {
        rsp_turtlebot3_msgs::add_scan_to_loop_detection_database srv;
        srv.request.scan = _curr_scan;
        srv.request.pose = _get_robot_pose(_curr_scan.header.stamp);
        _add_scan_client.call(srv);
    } else {
        _detect_loop = false;
        rsp_turtlebot3_msgs::detect_loop srv;
        srv.request.scan = _curr_scan;
        srv.request.pose = _get_robot_pose(_curr_scan.header.stamp);
        _loop_detection_client.call(srv);
        ROS_INFO_STREAM(
            "Detect loop: " << (srv.response.detected ? "Success" : "Failure"));
    }

    auto move_base_state = _move_base_client.getState();
    if (move_base_state == move_base_state.ACTIVE &&
        ros::Time::now() - _last_goal_start_time > ros::Duration(5.0)) {
        _move_base_client.cancelGoal();
        ROS_INFO_STREAM("Canceled goal");
    }

    if (_explore) {
        move_base_state = _move_base_client.getState();
        if (move_base_state != move_base_state.ACTIVE) {
            rsp_turtlebot3_msgs::get_frontier_exploration_goal srv;
            srv.request.map = _curr_map;
            srv.request.curr_pose = _get_robot_pose(_curr_map.header.stamp);

            if (_frontier_exlore_client.call(srv)) {
                const auto& pose = srv.response.goal_pose;
                ROS_INFO_STREAM("Explore goal pose [x, y, yaw] = "
                                << pose.x << " " << pose.y << " " << pose.yaw);
                _move_robot(pose);
            }
        }
    }
}

void RttTurtlebot3::stopHook() {}
void RttTurtlebot3::cleanupHook() {}

/* #################### RTT components callbacks #################### */

rsp_turtlebot3_msgs::rsp_turtlebot3_pose RttTurtlebot3::getRobotPoseClbk()
    const {
    return _get_robot_pose();
}
void RttTurtlebot3::setRobotPosClbk(
    const rsp_turtlebot3_msgs::rsp_turtlebot3_pose& robot_pose) {
    _move_robot(robot_pose);
}

void RttTurtlebot3::detectLoopClbk() { _detect_loop = true; }

void RttTurtlebot3::exploreClbk() { _explore = true; }

/* #################### subscribers callbacks #################### */

void RttTurtlebot3::scanSubClbk(const sensor_msgs::LaserScanConstPtr& msg) {
    _curr_scan = *msg;
}

void RttTurtlebot3::mapSubClbk(const nav_msgs::OccupancyGridConstPtr& msg) {
    _curr_map = *msg;
}

/* #################### helper functions #################### */

double get_yaw_from_quaternion(double x, double y, double z, double w) {
    tf::Quaternion tf_q(x, y, z, w);
    tf::Matrix3x3 rot_m(tf_q);
    double roll, pitch, yaw;
    rot_m.getRPY(roll, pitch, yaw);
    return yaw;
}

rsp_turtlebot3_msgs::rsp_turtlebot3_pose RttTurtlebot3::_get_robot_pose(
    ros::Time stamp) const {
    tf::StampedTransform transform;
    rsp_turtlebot3_msgs::rsp_turtlebot3_pose pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.yaw = 0.0;
    try {
        _tf_listener.waitForTransform("/map", "/base_footprint", stamp,
                                      ros::Duration(3.0));
        _tf_listener.lookupTransform("/map", "/base_footprint", stamp,
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

void RttTurtlebot3::_move_robot(
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

    _last_goal_start_time = ros::Time::now();

    _move_base_client.sendGoal(
        goal, boost::bind(&RttTurtlebot3::_move_base_done_clbk, this, _1, _2),
        boost::bind(&RttTurtlebot3::_move_base_active_clbk, this),
        boost::bind(&RttTurtlebot3::_move_base_feedback_clbk, this, _1));
}

/* #################### move base action client callbacks ####################
 */

void RttTurtlebot3::_move_base_done_clbk(
    const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result) const {
    ROS_INFO_STREAM("Finished moving robot");
}

void RttTurtlebot3::_move_base_active_clbk() const {}

void RttTurtlebot3::_move_base_feedback_clbk(
    const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) const {
    const double x = feedback->base_position.pose.position.x;
    const double y = feedback->base_position.pose.position.y;
    const auto q = feedback->base_position.pose.orientation;
    const double yaw = get_yaw_from_quaternion(q.x, q.y, q.z, q.w);

    // ROS_INFO_STREAM("Current robot pose [x, y, yaw] = " << x << " " << y << "
    // "
    // << yaw);
}

ORO_CREATE_COMPONENT(RttTurtlebot3);