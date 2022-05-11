/**
 * @file pathPlanning.hpp
 * @author Xiaorui Zhang, Xucheng Ma
 * @brief pathPlanning header file
 * @version 0.1
 * @date 2022-05-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <rsp_random_walk_frontier_exploration/collisionDetection.hpp>
class pathPlanning {
 private:
  // Create collision detection object
  collisionDetection detector;
  float linearSpeed;
  float angularSpeed;
  // Use twist to control the turtlebot
  geometry_msgs::Twist twist;
  ros::Publisher vel_publisher;
  ros::Subscriber laser_subscriber;
  ros::NodeHandle nh;

 public:
  pathPlanning();
  ~pathPlanning();
  void linearPathGenerator();
};