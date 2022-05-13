#include "rsp_random_walk_exploration/rsp_random_walk_exploration.hpp"

// Constructor
RandomWalkAction::RandomWalkAction(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
    // setup pulisher and subscriber
    scan_sub_ =
        nh_.subscribe("scan", 100, &RandomWalkAction::scanSubClbk, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    pubZeroTwist();

    // setup action server
    std::string random_walk_action;
    pnh_.param<std::string>("random_walk_action", random_walk_action,
                            "random_walk");
    as_.reset(new RandomWalkServer(
        pnh_, random_walk_action,
        boost::bind(&RandomWalkAction::executeClbk, this, _1), false));
    as_->start();

    // params for random walk
    pnh_.param<float>("linear_speed", linear_speed_, 0.2);
    pnh_.param<float>("angular_speed", angular_speed_, 1);
    pnh_.param<float>("threshold", threshold_, 0.5);
    pnh_.param<float>("bubble_size", bubble_size_, 0.4);
    pnh_.param<float>("CCW_prob", CCW_prob_, 0.5);
    pnh_.param<int>("planning_rate", rate_, 2);
}

// operational functions
// Stop the turtlebot
void RandomWalkAction::pubZeroTwist() const {
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    vel_pub_.publish(twist);
}
void RandomWalkAction::linearPathGenerator() const {
    geometry_msgs::Twist twist;
    switch (collision_region_) {
        case CollisionType::FrontCollsion:
            // obstacle in front of the turtlebot
            // 50% rotate CCW
            // ROS_INFO_STREAM("obstacle in front ");
            twist.linear.x = 0;
            if ((std::rand() / RAND_MAX) <= CCW_prob_) {
                twist.angular.z = angular_speed_;
            } else {
                twist.angular.z = -angular_speed_;
            }
            break;
        case CollisionType::RearCollision:
            // obstacle at the rear, keep moving
            // ROS_INFO_STREAM("obstacle in rear ");
            twist.linear.x = linear_speed_;
            twist.angular.z = 0.0;
            break;
        case CollisionType::NoCollision:
            // obstacle equal distance front and rear
            // ROS_INFO_STREAM("no obstacle ");
            twist.linear.x = linear_speed_;
            twist.angular.z = 0.0;
            break;
    }
    // Publish the twist
    vel_pub_.publish(twist);
}

// Callbacks
void RandomWalkAction::scanSubClbk(const sensor_msgs::LaserScanConstPtr& msg) {
    std::vector<float> range_vec = msg->ranges;
    // replace all the inf values by threshold
    std::replace(range_vec.begin(), range_vec.end(),
                 std::numeric_limits<float>::infinity(), threshold_);
    double sum_front = 0, sum_rear = 0;
    for (int idx = 0; idx < range_vec.size(); ++idx) {
        // ignore the value bigger than 0.5 (no collision beyond 0.5)
        if (range_vec[idx] >= 0.0 && range_vec[idx] <= bubble_size_) {
            // Get a rough feeling of where is the object, front (90 angles
            // 315-44), rear (angle 135-225)
            switch (idx) {
                case 0 ... 44:
                    sum_front += range_vec[idx];
                    break;
                case 315 ... 359:
                    sum_front += range_vec[idx];
                    break;
                case 135 ... 224:
                    sum_rear += range_vec[idx];
                    break;
            }
        }
    }
    // all inf is 0.5, if sum_front is smaller, then object is closer in the
    // front collision_region 1 for front, 2 for rear, 0 for both equally
    // distanced
    if (sum_front > sum_rear) {
        collision_region_ = CollisionType::FrontCollsion;
    }
    if (sum_rear > sum_front) {
        collision_region_ = CollisionType::RearCollision;
    }
    if (sum_rear == sum_front) {
        collision_region_ = CollisionType::NoCollision;
    }
}

void RandomWalkAction::executeClbk(
    const rsp_turtlebot3_msgs::RandomWalkGoalConstPtr& goal) {
    // Now the executeClbk function referenced in the constructor is created.
    // The callback function is passed a pointer to the goal message. Note: This
    // is a boost shared pointer, given by appending "ConstPtr" to the end of
    // the goal message type.
    ros::Time _CB_start_time = ros::Time::now();
    rsp_turtlebot3_msgs::RandomWalkFeedback randomWalk_feedback;
    ros::Rate rate(rate_);
    bool success = true;
    while (ros::Time::now() - _CB_start_time < ros::Duration(goal->goal_time)) {
        if (as_->isPreemptRequested() || !ros::ok()) {
            as_->setPreempted();
            success = false;
            break;
        }
        // feed back how long is running
        linearPathGenerator();
        randomWalk_feedback.progress =
            float((ros::Time::now() - _CB_start_time).toSec());
        as_->publishFeedback(randomWalk_feedback);
        rate.sleep();
    }
    pubZeroTwist();
    if (success) {
        as_->setSucceeded();
    }
}

RandomWalkAction::~RandomWalkAction() { pubZeroTwist(); }
