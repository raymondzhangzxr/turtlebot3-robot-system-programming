#include "rsp_random_walk_exploration/rsp_random_walk_exploration.hpp"

// Constructor
RandomWalk::RandomWalk(ros::NodeHandle& nh) : _nh(nh){
        ROS_INFO("randomWalk action constructed");
        _scan_sub = _nh.subscribe("scan", 100, &RandomWalk::scanSubCB, this);
        _vel_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        zeroTwist();
        _linearSpeed = 0.2;
        _angularSpeed = 1;
        
        _as.reset(new _RandomWalkServer(_nh,"RandomWalkAction",boost::bind(&RandomWalk::executeCB, this, _1),false));
        _as->start();
}

// operational functions
// Stop the turtlebot
void RandomWalk::zeroTwist(){
    _twist.linear.x = 0.0;
    _twist.linear.y = 0.0;
    _twist.linear.z = 0.0;
    _twist.angular.x = 0.0;
    _twist.angular.y = 0.0;
    _twist.angular.z = 0.0;
    _vel_pub.publish(_twist);
}
void RandomWalk::linearPathGenerator()
{   
    switch (_collision_region){
        case 1:
            // obstacle in front of the turtlebot
            // 50% rotate CCW
            ROS_INFO_STREAM("obstacle in front ");
            _twist.linear.x = 0;
            if((std::rand()/RAND_MAX) <= 0.5){
                _twist.angular.z = _angularSpeed;
            }
            else{
                _twist.angular.z = -_angularSpeed;
            }
            break;
        case 2:
            // obstacle at the rear, keep moving
            ROS_INFO_STREAM("obstacle in rear ");
            _twist.linear.x = _linearSpeed;
            _twist.angular.z = 0.0;
            break;
        case 0:
            // obstacle equal distance front and rear
            ROS_INFO_STREAM("no obstacle ");
            _twist.linear.x = _linearSpeed;
            _twist.angular.z = 0.0;
            break;
    }
    // Publish the twist
     _vel_pub.publish(_twist);
}

// Callbacks
void RandomWalk::scanSubCB(const sensor_msgs::LaserScanConstPtr& msg){
    std::vector<float> range_vec = msg->ranges;
    // replace all the inf values by 0.5
    float limit = 0.5;
    std::replace(range_vec.begin(),range_vec.end(),std::numeric_limits<float>::infinity(), limit);
    double sum_front = 0, sum_rear = 0;
    for (int idx = 0; idx < range_vec.size(); ++idx){
        // ignore the value bigger than 0.5 (no collision beyond 0.5)
        if(range_vec[idx]>=0.0 && range_vec[idx]<=0.4){
            // Get a rough feeling of where is the object, front (90 angles 315-44), rear (angle 135-225)
            switch (idx){
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
    // all inf is 0.5, if sum_front is smaller, then object is closer in the front
    // collision_region 1 for front, 2 for rear, 0 for both equally distanced
    if (sum_front > sum_rear){
        _collision_region = 1; 
    }
    if (sum_rear > sum_front){
        _collision_region = 2; 
    }
    if (sum_rear == sum_front){
        _collision_region = 0;
    }
}

void RandomWalk::executeCB(const rsp_turtlebot3_msgs::RandomWalkGoalConstPtr &goal){
    // Now the executeCB function referenced in the constructor is created. The callback function is passed a pointer to the goal message. Note: This is a boost shared pointer, given by appending "ConstPtr" to the end of the goal message type.
   std::cout <<"RandomWalk callback"<< std::endl;
   ros::Time _CB_start_time = ros::Time::now();
   rsp_turtlebot3_msgs::RandomWalkFeedback randomWalk_feedback;
   while(ros::Time::now() - _CB_start_time < ros::Duration(goal->goal_time) ){
        ros::Rate rate(2);
       // feed back how long is running
        linearPathGenerator();
        randomWalk_feedback.progress = float((ros::Time::now() - _CB_start_time).toSec());
        _as->publishFeedback(randomWalk_feedback);
        std::cout <<"The progress is: "+ std::to_string(randomWalk_feedback.progress) << std::endl;
        rate.sleep();
   }
   zeroTwist();
   _as->setSucceeded();
}

RandomWalk::~RandomWalk(){
    zeroTwist();
}
