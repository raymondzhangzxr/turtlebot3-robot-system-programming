# Turtlebot3 frontier exploration and loop detection
<h3 align="center">EN.530.707 Robot System Programming class project. </h3>
<h4 align="center">Xiaorui Zhang (xzhan227)
, Xucheng Ma (xma42)</h4>
<h4 align="center">Johns Hopkins Robotics</h4>

## Table of contents
- [Description](#description)
- [Deliverable checklist](#deliverable-checklist)
- [Getting started](#getting-started)
- [What's included](#whats-included)
- [Authors](#authors)
- [Acknowledgments](#acknowledgments)

## Description

This project implements two frontier exploration methods (Wavefront and Random walk) with loop detection on the Turtlebot3 platform. We focused on integrating SLAM algorithms and ROS by applying in-class learned knowledge and skills. We did both Gazebo simulation and real-world robot experiments in this project.

## Deliverable checklist
- [x] **Minimum Deliverbles**
- A working SLAM algorithm in both Gazebo Simulation and real world implementation.
- [x] **Expected Deliverbles**
- A working SLAM algorithm with loop detection in both Gazebo Simulation and real world implementation.
- [x] **Maximum Deliverables**
- A working SLAM algorithm with loop detection in both Gazebo Simulation and real world implementation. Plus custome robot manipulation through ROS actions and services. 
## Getting Started

### Installing the packages
This package requires Orocos RTT, if you have it already skip the RTT installation part. 
* Go to your favorite catkin workspace or create one
```bash
# Create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws/src
```
* Clone RTT:
```bash
# Clone the following packages
$ git clone https://github.com/orocos-toolchain/orocos_toolchain.git --recursive
$ git clone https://github.com/orocos/rtt_ros_integration.git rtt_ros_integration
$ git clone https://github.com/orocos/rtt_geometry.git
```
* Clone our package:
```bash
# Clone
$ git clone https://github.com/raymondzhangzxr/RSP_Project.git

# Update submodules
$ cd RSP_Project
$ git submodule update --init --recursive
```

### Building the packages
Takes some time if you don't have RTT.
```bash
# catkin build
$ cd ../../ && catkin build 
```
### Launching the packages
#### Gazebo Simulation
```bash
# source and launch
$ source devel/setup.bash 
$ export TURTLEBOT3_MODEL=burger
$ roslaunch rsp_turtlebot3_slam rsp_turtlebot3_simulation.launch 
```
In a seperate terminal Launch RTT deployer interface:
```bash
# source and launch
$ source devel/setup.bash 
$ roslaunch rsp_turtlebot3_slam rsp_turtlebot3_interface.launch
```
#### Turtlebot3 Demo
TODO
* Launch turtlebot3 demo:
```bash
# You need to first bring up turtlebot3 then run  
$ roslaunch rsp_turtlebot3_slam TODO
``` 


### Using the rtt deployer:
Once the deployer is launched, first switch to rsp_slam
```bash
TODO
```
To perform different taskes, run the following command inside the deplyer
* To start auto frontier exploration
```bash
Explore
```
* To stop exploration at any time 
```bash
Stop
```
* To run random walk obstacle avoidance exploration 
```bash
# take running duration as input 
RandomWalk(10.0)
```
* To get current robot pose 
```bash
GetRobotPose
```
* To set a robot pose 
```bash
SetRobotPose
```

* To check for loop closure
```bash
LoopDetection
```


## What's included
```text
RSP_project/
├── rsp_loop_detection/
├── rsp_random_walk_exploration/
├── rsp_wf_frontier_exploration/
├── rsp_turtlebot3_msgs/
├── rsp_turtlebot3_slam/
├── rtt_rsp_turtlebot3_msgs/
├── turtlebot3/
└── turtlebot3_simulations/
```
#### rsp_loop_detection:
A 2D loop closure detection **ros service** to compare current map with ones in the database to determine whether the turtlebot has been visited this place. 
#### rsp_random_walk_exploration:
A **ros action** for auto obstacle avoidance exploration. Turtlebot randomly turns around when there is an obstacle in front/rear. 
#### rsp_wf_frontier_exploration:
A **ros service** for auto wavefront frontier exploration. Turtlebot search for frontier points in current map using BFS.  
#### rsp_turtlebot3_msgs:
Srv, msgs, and action files for this package.
```text
rsp_turtlebot3_msgs/
├── action/
│        └── Randomwalk.action
├── msg/
│     └── rsp_turtlebot_pose.msg      
└── srv/  
      ├── add_scan_to_loop_detection_database.srv
      ├── detect_loop.srv
      └── get_frontier_exploration_goal.srv     
```
#### rtt_rsp_turtlebot3_msgs
Generate rtt required msgs and services. 
#### rsp_turtlebot3_slam:
The main package for gazebo simulation, rtt deployer, and turtlebot3 demo. 

## Authors
* [@Xiaorui (Ray) Zhang](https://github.com/raymondzhangzxr)
* [@Xucheng Ma](https://github.com/mxchenggggg)

## Demo Videos
* Video 1:
  
* Video 2:

## Acknowledgments

This package used the following packages as submodules. 
* [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
* [turtlebot3 (forked and modified)](https://github.com/mxchenggggg/turtlebot3)

Wavefront frontier exploration inspired by:  
* [turtlebot_slam](https://github.com/tpepels/turtlebot_slam)
