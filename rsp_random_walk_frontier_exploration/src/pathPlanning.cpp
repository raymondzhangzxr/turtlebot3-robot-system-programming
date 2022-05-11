/**
 * @file pathPlanning.cpp
 * @author Xiaorui Zhang, Xucheng Ma
 * @brief
 * @version 0.1
 * @date 2022-05-07
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <rsp_random_walk_frontier_exploration/pathPlanning.hpp>

pathPlanning::pathPlanning()
{   ROS_INFO_STREAM("path planning constructor");
    // Set speed parameters
    linearSpeed = 0.2;
    angularSpeed = 1;
    // register to publish topic on /cmd_vel
    // to send move velocity commands to the turtlebot
    vel_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    // creating Subscriber sub  subscribing to scan topic and calling
    // lasercallback function of CollisionDetector class
    laser_subscriber = nh.subscribe<sensor_msgs::LaserScan>("/scan", 50,
                                                            &collisionDetection::laserCallback,
                                                            &detector);
    // Define the initial velocity message
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    // Stop the turtlebot
    vel_publisher.publish(twist);
}

pathPlanning::~pathPlanning()
{
    // Stop the turtlebot before exiting
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    // Stop the turtlebot
    vel_publisher.publish(twist);
}

void pathPlanning::linearPathGenerator()
{
    ros::Rate rate(2);
    std::cout<<"It's working"<<std::endl;
    //  check where is the obstacle and move accordingly
    switch (detector.where_collision()){
        case 1:
            // obstacle in front of the turtlebot
            // 50% rotate CCW
            ROS_INFO_STREAM("obstacle in front ");
            twist.linear.x = 0;
            if((std::rand()/RAND_MAX) <= 0.5){
                twist.angular.z = angularSpeed;
            }
            else{
                twist.angular.z = -angularSpeed;
            }
            break;
        case 2:
            // obstacle at the rear, keep moving
            ROS_INFO_STREAM("obstacle in rear ");
            twist.linear.x = linearSpeed;
            twist.angular.z = 0.0;
            break;
        case 0:
            // obstacle equal distance front and rear
            ROS_INFO_STREAM("no obstacle ");
            twist.linear.x = linearSpeed;
            twist.angular.z = 0.0;
            break;
    }
    // Publish the twist
    vel_publisher.publish(twist);
    ros::spinOnce();
    // Sleep for the remaining time until we hit our 2 Hz rate
    rate.sleep();
}
