/**
 * @file collisionDetection.cpp
 * @author Xiaorui Zhang, Xucheng Ma
 * @brief  Naive Collision Detection implementation
 * @version 0.1
 * @date 2022-05-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <rsp_frontier_exploration/collisionDetection.hpp>

collisionDetection::collisionDetection() {
  ROS_INFO("Initializing Collision Detection!");
  collision_region = 0;
}

collisionDetection::~collisionDetection(){}

void collisionDetection::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // save ranges into a vector
  std::vector<float> range_vec = msg->ranges;
  // replace all the inf values by 0.5
  float limit = 0.5;
  std::replace(range_vec.begin(),range_vec.end(),std::numeric_limits<float>::infinity(), limit);
  double sum_front = 0, sum_rear = 0;

  for (int idx = 0; idx < range_vec.size(); ++idx){
      // ignore the value bigger than 0.5 (no collision beyond 0.5)
      if(range_vec[idx]>=0.0 && range_vec[idx]<=0.5){
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
  ROS_INFO_STREAM("Front area: " << sum_front);
  ROS_INFO_STREAM("Rear area: " << sum_rear);
  // all inf is 0.5, if sum_front is smaller, then object is closer in the front
  // collision_region 1 for front, 2 for rear, 0 for both equally distanced
  if (sum_front < 5*sum_rear){
      collision_region = 1; 
  }
  if (sum_rear < 5*sum_front){
      collision_region = 2; 
  }
  if (sum_rear == sum_front){
      collision_region = 0;
  }
}

int collisionDetection::where_collision(){
  return collision_region;
}
