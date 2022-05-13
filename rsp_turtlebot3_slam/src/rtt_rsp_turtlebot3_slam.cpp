#include "rsp_turtlebot3_slam/rtt_rsp_turtlebot3_slam.hpp"

#include <ros/ros.h>

#include "rsp_turtlebot3_msgs/RandomWalkAction.h"
#include "rsp_turtlebot3_msgs/add_scan_to_loop_detection_database.h"
#include "rsp_turtlebot3_msgs/detect_loop.h"
#include "rsp_turtlebot3_msgs/get_frontier_exploration_goal.h"
#include "rtt/Component.hpp"

RttTurtlebot3::RttTurtlebot3(const std::string& name)
    : RTT::TaskContext(name),
      detect_loop_(false),
      explore_(false),
      random_walk_(false) {
    // setup operations
    addOperation("GetRobotPose", &RttTurtlebot3::getRobotPoseClbk, this,
                 RTT::OwnThread);
    addOperation("SetRobotPose", &RttTurtlebot3::setRobotPosClbk, this,
                 RTT::OwnThread);
    addOperation("LoopDetection", &RttTurtlebot3::detectLoopClbk, this,
                 RTT::OwnThread);
    addOperation("Explore", &RttTurtlebot3::exploreClbk, this, RTT::OwnThread);

    addOperation("RandomWalk", &RttTurtlebot3::randomWalkClbk, this,
                 RTT::OwnThread);

    addOperation("Stop", &RttTurtlebot3::stopClbk, this, RTT::OwnThread);

    // subscriber topic names
    addProperty("scan_topic", scan_topic_);
    addProperty("map_topic", map_topic_);

    // loop detection service names
    addProperty("add_scan_service_name", add_scan_service_name_);
    addProperty("detect_loop_service_name", detect_loop_service_name_);

    // get wavefront frontire exploration goal service name
    addProperty("get_exp_goal_service_name", get_exp_goal_service_name_);

    // random walk action name
    addProperty("random_walk_action_name", random_walk_action_name_);

    // move base action name
    addProperty("move_base_action_name", move_base_action_name_);

    // frame ids for robot pose tf look up
    addProperty("world_frame", world_frame_);
    addProperty("robot_frame", robot_frame_);

    addProperty("move_base_goal_wait_time", move_base_goal_wait_time_);

    addProperty("random_walk_time_when_stuck", random_walk_time_when_stuck_);
}

RttTurtlebot3::~RttTurtlebot3() {}

/* #################### RTT hooks #################### */
bool RttTurtlebot3::configureHook() {
    // setup subscribers
    scan_sub_ =
        nh_.subscribe(scan_topic_, 1000, &RttTurtlebot3::scanSubClbk, this);
    map_sub_ =
        nh_.subscribe(map_topic_, 1000, &RttTurtlebot3::mapSubClbk, this);

    // loop detection clients
    add_scan_client_ = nh_.serviceClient<
        rsp_turtlebot3_msgs::add_scan_to_loop_detection_database>(
        add_scan_service_name_);
    ROS_INFO("Waiting for add scan service to start");
    add_scan_client_.waitForExistence();

    loop_detection_client_ =
        nh_.serviceClient<rsp_turtlebot3_msgs::detect_loop>(
            detect_loop_service_name_);
    ROS_INFO("Waiting for loop detect service to start");
    loop_detection_client_.waitForExistence();

    // frontier exploration client
    frontier_exlore_client_ =
        nh_.serviceClient<rsp_turtlebot3_msgs::get_frontier_exploration_goal>(
            get_exp_goal_service_name_);
    ROS_INFO("Waiting for get frontier exploration goal service to start");
    frontier_exlore_client_.waitForExistence();

    // random walk action client
    random_walk_client_.reset(
        new RandomWalkActionClient(random_walk_action_name_, true));
    ROS_INFO("Waiting for random walk action server to start");
    random_walk_client_->waitForServer();

    // move base action client
    move_base_client_.reset(
        new MoveBaseActionClient(move_base_action_name_, true));
    ROS_INFO("Waiting for move base action server to start");
    move_base_client_->waitForServer();
}
bool RttTurtlebot3::startHook() { return true; }

void RttTurtlebot3::updateHook() {
    if (!detect_loop_) {
        rsp_turtlebot3_msgs::add_scan_to_loop_detection_database srv;
        srv.request.scan = curr_scan_;
        srv.request.pose = getRobotPose(curr_scan_.header.stamp);
        add_scan_client_.call(srv);
    } else {
        detect_loop_ = false;
        rsp_turtlebot3_msgs::detect_loop srv;
        srv.request.scan = curr_scan_;
        srv.request.pose = getRobotPose(curr_scan_.header.stamp);
        loop_detection_client_.call(srv);
        ROS_INFO_STREAM(
            "Detect loop: " << (srv.response.detected ? "Success" : "Failure"));
    }

    const auto move_base_state = move_base_client_->getState();
    if (move_base_state == move_base_state.ACTIVE &&
        ros::Time::now() - last_goal_start_time_ >
            ros::Duration(move_base_goal_wait_time_)) {
        move_base_client_->cancelGoal();
        if (explore_) {
            ROS_INFO_STREAM("Switched to random walk.");
            randomWalk(random_walk_time_when_stuck_);
        }
    }

    if (explore_) {
        const auto move_base_state = move_base_client_->getState();
        const auto random_walk_state = random_walk_client_->getState();
        if (move_base_state != move_base_state.ACTIVE &&
            random_walk_state != random_walk_state.ACTIVE) {
            rsp_turtlebot3_msgs::get_frontier_exploration_goal srv;
            srv.request.map = curr_map_;
            srv.request.curr_pose = getRobotPose(curr_map_.header.stamp);

            if (frontier_exlore_client_.call(srv)) {
                const auto& pose = srv.response.goal_pose;
                ROS_INFO_STREAM("Exploration goal pose [x, y, yaw] = "
                                << pose.x << " " << pose.y << " " << pose.yaw);
                moveRobot(pose);
            }
        }
    }

    if (random_walk_) {
        const auto random_walk_state = random_walk_client_->getState();
        if (random_walk_state != random_walk_state.ACTIVE) {
            randomWalk(random_walk_time_);
        }
    }
}

void RttTurtlebot3::stopHook() {}
void RttTurtlebot3::cleanupHook() {}

/* #################### RTT components callbacks #################### */

rsp_turtlebot3_msgs::rsp_turtlebot3_pose RttTurtlebot3::getRobotPoseClbk()
    const {
    return getRobotPose();
}
void RttTurtlebot3::setRobotPosClbk(
    const rsp_turtlebot3_msgs::rsp_turtlebot3_pose& robot_pose) {
    moveRobot(robot_pose);
}

void RttTurtlebot3::detectLoopClbk() { detect_loop_ = true; }

void RttTurtlebot3::exploreClbk() {
    if (random_walk_) {
        ROS_INFO_STREAM(
            "Please stop the robot before starting a new exploration.");
        explore_ = false;
    } else {
        explore_ = true;
    }
}

void RttTurtlebot3::randomWalkClbk(double running_time) {
    if (explore_) {
        ROS_INFO_STREAM(
            "Please stop the robot before starting a new random walk.");
        random_walk_ = false;
    } else {
        random_walk_ = true;
        random_walk_time_ = running_time;
    }
}

void RttTurtlebot3::stopClbk() {
    random_walk_ = false;
    explore_ = false;
    const auto move_base_state = move_base_client_->getState();
    if (move_base_state == move_base_state.ACTIVE) {
        move_base_client_->cancelGoal();
    }
    const auto random_walk_state = random_walk_client_->getState();
    if (random_walk_state == random_walk_state.ACTIVE) {
        random_walk_client_->cancelGoal();
    }
}

/* #################### subscribers callbacks #################### */

void RttTurtlebot3::scanSubClbk(const sensor_msgs::LaserScanConstPtr& msg) {
    curr_scan_ = *msg;
}

void RttTurtlebot3::mapSubClbk(const nav_msgs::OccupancyGridConstPtr& msg) {
    curr_map_ = *msg;
}

/* #################### helper functions #################### */

double get_yaw_from_quaternion(double x, double y, double z, double w) {
    tf::Quaternion tf_q(x, y, z, w);
    tf::Matrix3x3 rot_m(tf_q);
    double roll, pitch, yaw;
    rot_m.getRPY(roll, pitch, yaw);
    return yaw;
}

rsp_turtlebot3_msgs::rsp_turtlebot3_pose RttTurtlebot3::getRobotPose(
    ros::Time stamp) const {
    tf::StampedTransform transform;
    rsp_turtlebot3_msgs::rsp_turtlebot3_pose pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.yaw = 0.0;
    try {
        tf_listener_.waitForTransform(world_frame_, robot_frame_, stamp,
                                      ros::Duration(3.0));
        tf_listener_.lookupTransform(world_frame_, robot_frame_, stamp,
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

void RttTurtlebot3::moveRobot(
    const rsp_turtlebot3_msgs::rsp_turtlebot3_pose& robot_pose) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = world_frame_;
    goal.target_pose.pose.position.x = robot_pose.x;
    goal.target_pose.pose.position.y = robot_pose.y;

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, robot_pose.yaw);

    goal.target_pose.pose.orientation.w = q.w();
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();

    last_goal_start_time_ = ros::Time::now();

    move_base_client_->sendGoal(
        goal, boost::bind(&RttTurtlebot3::moveBaseDoneClbk, this, _1, _2),
        boost::bind(&RttTurtlebot3::moveBaseActiveClbk, this),
        boost::bind(&RttTurtlebot3::moveBaseFeedbackClbk, this, _1));
}

void RttTurtlebot3::randomWalk(double time) {
    rsp_turtlebot3_msgs::RandomWalkGoal rw_goal;
    rw_goal.goal_time = time;
    random_walk_client_->sendGoal(
        rw_goal, boost::bind(&RttTurtlebot3::randomWalkDoneClbk, this, _1, _2),
        boost::bind(&RttTurtlebot3::randomWalkActiveClbk, this),
        boost::bind(&RttTurtlebot3::randomWalkFeedbackClbk, this, _1));
}

/* #################### move base action client callbacks ################### */

void RttTurtlebot3::moveBaseDoneClbk(
    const actionlib::SimpleClientGoalState& state,
    const move_base_msgs::MoveBaseResultConstPtr& result) const {
    ROS_INFO_STREAM("Finished moving robot");
}

void RttTurtlebot3::moveBaseActiveClbk() const {}

void RttTurtlebot3::moveBaseFeedbackClbk(
    const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) const {
    const double x = feedback->base_position.pose.position.x;
    const double y = feedback->base_position.pose.position.y;
    const auto q = feedback->base_position.pose.orientation;
    const double yaw = get_yaw_from_quaternion(q.x, q.y, q.z, q.w);

    // ROS_INFO_STREAM("Current robot pose [x, y, yaw] = " << x << " " << y << "
    // "
    // << yaw);
}

void RttTurtlebot3::randomWalkDoneClbk(
    const actionlib::SimpleClientGoalState& state,
    const rsp_turtlebot3_msgs::RandomWalkResultConstPtr& result) {
    ROS_INFO_STREAM("Finished random walk");
    random_walk_ = false;
}

void RttTurtlebot3::randomWalkActiveClbk() const {}

void RttTurtlebot3::randomWalkFeedbackClbk(
    const rsp_turtlebot3_msgs::RandomWalkFeedbackConstPtr& feedback) const {
    // ROS_INFO_STREAM("The progress is: " <<
    // std::to_string(feedback->progress));
}

ORO_CREATE_COMPONENT(RttTurtlebot3);