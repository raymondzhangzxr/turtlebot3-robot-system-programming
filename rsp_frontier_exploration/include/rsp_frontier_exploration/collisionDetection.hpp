/**
 * @file collisionDetection.hpp
 * @author Xiaorui Zhang, Xucheng Ma
 * @brief Naive Collision Detection header file
 * @version 0.1
 * @date 2022-05-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <vector>
#include <algorithm>


class collisionDetection{
    private: 
        int collision_region;
    public: 
        collisionDetection();
        ~collisionDetection();
        // change all inf to 0.5, determine where is closer to a collision
        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        // return where is collision front (return 1) or rear (return 2) is closer to a collision
        int where_collision();
};