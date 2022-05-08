/**
 * @file main.cpp
 * @author Xiaorui Zhang, Xucheng Ma
 * @brief  main file for the frontier exploration
 * @version 0.1
 * @date 2022-05-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>
#include <iostream>
#include <rsp_frontier_exploration/pathPlanning.hpp>
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "frontierExplorer");
  // Variable to store user input
  std::cout << "Once you are satisfied with the map press (ctr+c)" << std::endl;
  std::cout << "To save the map rosrun map_server map_saver -f my_map"
            << std::endl;
 
  pathPlanning pathPlanning;
  while (ros::ok()) {
      pathPlanning.linearPathGenerator();
  }
  return 0;
}
