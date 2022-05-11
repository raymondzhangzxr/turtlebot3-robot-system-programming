#include <rsp_random_walk_frontier_exploration/randomWalk_action.hpp>
#include <rsp_random_walk_frontier_exploration/pathPlanning.hpp>

// Constructor
randomWalk_action::randomWalk_action( ros::NodeHandle& nh):
    
    // this creates a local NodeHandle variable within the class that is a a copy of the NodeHandle
    nh(nh){
        ROS_INFO("randomWalk action constructed");
        //reset the pointer on heap (global) Server is defined in hpp
        as.reset( new RandomWalkServer(nh,"RandomWalkAction",boost::bind(&randomWalk_action::executeCB, this, _1),false));
        // register CB use boost library 
        // give it the address of function
        // Execute_Callback: Optional callback that gets called in a separate thread whenever a new goal is received, allowing users to have blocking callbacks. Adding an execute callback also deactivates the goalCallback.
        // start the server by calling the function start
        as->start();// this will create a bunch of topics: goal,feedback, result
}

void randomWalk_action::executeCB(const rsp_turtlebot3_msgs::RandomWalkGoalConstPtr &goal){
    // Now the executeCB function referenced in the constructor is created. The callback function is passed a pointer to the goal message. Note: This is a boost shared pointer, given by appending "ConstPtr" to the end of the goal message type.
   std::cout <<"RandomWalk is working"<< std::endl;
   ros::Time _CB_start_time = ros::Time::now();
   rsp_turtlebot3_msgs::RandomWalkFeedback randomWalk_feedback;
   pathPlanning pathPlanning;

   while(ros::Time::now() - _CB_start_time < ros::Duration(goal->goal_time) ){
       // feed back how long is running
        pathPlanning.linearPathGenerator();
        randomWalk_feedback.progress = float((ros::Time::now() - _CB_start_time).toSec());
        as->publishFeedback(randomWalk_feedback);
        std::cout <<"The progress is: "+ std::to_string(randomWalk_feedback.progress) << std::endl;
   }
   as->setSucceeded();
}

randomWalk_action::~randomWalk_action(){}