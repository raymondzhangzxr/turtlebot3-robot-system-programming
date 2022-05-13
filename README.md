# Turtlebot3 SLAM with frontier exploration and loop detection
<h3 align="center">EN.530.707 Robot System Programming class project</h3>
<h4 align="center">Xiaorui Zhang (xzhan227); Xucheng Ma (xma42)</h4>
<h4 align="center">Johns Hopkins Robotics</h4>
<p align="center">
  <img src="teaser.gif" alt="gif" width="700" />
</p>

## Table of contents
- [Description](#description)
- [Deliverable checklist](#deliverable-checklist)
- [Getting started](#getting-started)
- [What's included](#whats-included)
- [Demo videos](#demo-videos)
- [Authors](#authors)
- [Acknowledgments](#acknowledgments)


## Description

This project implements two frontier exploration methods (Wavefront and Random walk) with loop detection on the Turtlebot3 platform.  Exploration and loop detection can be controlled in real time through Orocos Real-Time Toolkit (RTT). We focused on integrating SLAM algorithms and ROS by applying in-class learned knowledge and skills. Both Gazebo simulation and real-world robot experiments were performed in this project.

## Deliverable checklist
- [x] **Minimum Deliverbles**
- A working SLAM algorithm in both Gazebo Simulation and real world implementation.
- [x] **Expected Deliverbles**
- A working SLAM algorithm with loop detection in both Gazebo Simulation and real world implementation.
- [x] **Maximum Deliverables**
- A working SLAM algorithm with loop detection in both Gazebo Simulation and real world implementation. Plus custom robot tasks through ROS actions and services. We implemented two exploration algorithms for the robot to explore the environment automatically.
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

# Update submodules (modified turtlebot3 and turtlebot_simulation)
$ cd RSP_Project
$ git submodule update --init --recursive
```
* Install dwa-local-planner
```bash
$ sudo apt install ros-melodic-dwa-local-planner
```

### Building the packages
Takes some time if you don't have RTT.
```bash
# catkin build
$ cd ../../ && catkin build 
```
### Runing simulation and demo on real robot
Gazebo Simulation
* First, start gazebo simulation:
```bash
# source and launch
$ source devel/setup.bash 
$ export TURTLEBOT3_MODEL=burger
$ roslaunch rsp_turtlebot3_slam rsp_turtlebot3_simulation.launch 
```
* Then, in a new terminal, launch RTT deployer interface:
```bash
# source and launch
$ source devel/setup.bash 
$ roslaunch rsp_turtlebot3_slam rsp_turtlebot3_interface.launch
```
Refer to "Using the rtt deployer" section below to run automatic exploration and other RTT operations
#### Turtlebot3 Demo with real robot
* Start roscore on local machine:
```bash
$ roscore
``` 
* SSH into turtlebot and bring up the robot:
```bash
# you need to set up ROS_HOSTNAME and ROS_MASTER_URI properly on turtlebot 
$ ssh pi@{IP_ADDRESS_OF_RASPBERRY_PI}
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
``` 
* Start SLAM on local machine:
```bash
$ roslaunch rsp_turtlebot3_slam rsp_turtlebot3_slam.launch
``` 
* Then, in a new terminal, launch RTT deployer interface:
```bash
# source and launch
$ source devel/setup.bash 
$ roslaunch rsp_turtlebot3_slam rsp_turtlebot3_interface.launch
```

### Using the rtt deployer:
Once the deployer is launched, first switch to rtt_rsp_turtlebot3_slam

```bash
cd rtt_rsp_turtlebot3_slam
```
Following operations are provided
* To start auto exploration with wave front frontier algorithm
```bash
Explore
```
* To run random walk obstacle avoidance exploration 
```bash
# take running duration[second] as input 
RandomWalk(10.0)
```
* To stop exploration at any time 
```bash
Stop
```
* To get current robot pose 
```bash
var rsp_turtlebot3_msgs.rsp_turtlebot3_pose p = GetRobotPose()
```
* To set a robot pose 
```bash
# it's recommended to move to position close to current robot position, 
# otherwise the planner may find it hard to find a path
var rsp_turtlebot3_msgs.rsp_turtlebot3_pose p = GetRobotPose()
p.x = p.x + 0.15
p.y = p.y + 0.15
p.yaw = p.yaw + 0.2
SetRobotPose(p)
```

* To check for loop closure. The loop detection algorithm is not very robust since it depends on some const threshold of the similarity score, a lot of failures as detection result are expected. One can also compare the pose of query scan and matched scan to determine whether it's a loop closure.
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
### rsp_loop_detection:
A 2D loop closure detection **ros service** to compare current map with ones in the database to determine whether the turtlebot has been visited this place. While the robot is exploring the environment, a database of scans will be built; when LoopDetection is called, the loop detector server will search the database for a scan that's most similar to current, a loop closure is detected if the similarity score is good enough. 
### rsp_random_walk_exploration:
A **ros action** for auto obstacle avoidance exploration. Turtlebot randomly turns around when there is an obstacle in front/rear. 
### rsp_wf_frontier_exploration:
A **ros service** for auto wavefront frontier exploration. Turtlebot search for frontier points in current map using BFS.  
### rsp_turtlebot3_msgs:
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
### rtt_rsp_turtlebot3_msgs
RTT generated package by running:
```bash
rosrun rtt_roscomm create_rtt_msgs rsp_turtlebot3_msgs
```
this makes rsp_turtlebot3_msgs available in RTT deployer
### rsp_turtlebot3_slam:
The main package for gazebo simulation, RTT deployer, and turtlebot3 demo. 

### turtlebot3:
Forked from original [turlebot3 repo](https://github.com/ROBOTIS-GIT/turtlebot3), modified navigation stack to run SLAM with auto-exploration.

### turtlebot3_simulations:
Cloned from [turtlebot_simulation repo](https://github.com/ROBOTIS-GIT/turtlebot3_simulations).

## Authors
* [@Xiaorui (Ray) Zhang](https://github.com/raymondzhangzxr)
* [@Xucheng Ma](https://github.com/mxchenggggg)

## Demo Videos
* Video 1:
https://drive.google.com/file/d/1357kR66ey3R1z6fbebZTqeJKhuZ3ZSxf/view?usp=sharing
* Video 2:
https://drive.google.com/file/d/1Wsf-ze6sHjchfZNfuJ4QruEkLyDgjb6h/view?usp=sharing


## Acknowledgments

This package used the following packages as submodules. 
* [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
* [turtlebot3 (forked and modified)](https://github.com/mxchenggggg/turtlebot3)

Wavefront frontier exploration inspired by:  
* [turtlebot_slam](https://github.com/tpepels/turtlebot_slam)
