# Turtlebot3 frontier exploration and loop detection

EN.530.707 Robot System Programming class project. Johns Hopkins Robotics.

## Table of contents
- [Description](#description)
- [Quick start](#quick-start)
- [What's included](#whats-included)
- [Bugs and feature requests](#bugs-and-feature-requests)
- [Contributing](#contributing)
- [Creators](#creators)
- [Thanks](#thanks)
- [Copyright and license](#copyright-and-license)

## Description

This project implements two frontier exploration methods (Wavefront and Random walk) with loop detection on the Turtlebot3 platform. We focused on integrating SLAM algorithms and ROS by applying in-class learned knowledge and skills. We did both Gazebo simulation and real-world robot experiments in this project.

## Getting Started
This package requires Orocos RTT, if you have it already skip the RTT installation part. 
### Installing the packages
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

* Launch Gazebo simulation:
```bash
# source and launch
$ source devel/setup.bash 
$ export TURTLEBOT3_MODEL=burger
$ roslaunch rsp_turtlebot3_slam rsp_turtlebot3_simulation.launch 
```
* In a seperate terminal Launch RTT deployer interface:
```bash
# source and launch
$ source devel/setup.bash 
$ roslaunch rsp_turtlebot3_slam rsp_turtlebot3_interface.launch
```
* Launch turtlebot3 demo:
```bash
# You need to first bring up turtlebot3 then run  
$ roslaunch rsp_turtlebot3_slam rsp_turtlebot3_interface.launch
```

## What's included

Some text

```text
folder1/
└── folder2/
    ├── folder3/
    │   ├── file1
    │   └── file2
    └── folder4/
        ├── file3
        └── file4
```


## Authors

* [@Xiaorui Zhang](https://github.com/raymondzhangzxr)
* [@Xucheng Ma](https://github.com/mxchenggggg)
## Version History

* 0.2
    * Various bug fixes and optimizations
    * See [commit change]() or See [release history]()
* 0.1
    * Initial Release

## License

This project is licensed under the [NAME HERE] License - see the LICENSE.md file for details

## Acknowledgments

Inspiration, code snippets, etc.
* [awesome-readme](https://github.com/matiassingers/awesome-readme)
* [PurpleBooth](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2)
* [dbader](https://github.com/dbader/readme-template)
* [zenorocha](https://gist.github.com/zenorocha/4526327)
* [fvcproductions](https://gist.github.com/fvcproductions/1bfc2d4aecb01a834b46)