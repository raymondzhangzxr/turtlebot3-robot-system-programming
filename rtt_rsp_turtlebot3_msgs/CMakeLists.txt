cmake_minimum_required(VERSION 2.8.3)
project(rtt_rsp_turtlebot3_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm)

ros_generate_rtt_typekit(rsp_turtlebot3_msgs)
ros_generate_rtt_service_proxies(rsp_turtlebot3_msgs)

orocos_generate_package(
  DEPENDS rsp_turtlebot3_msgs
  DEPENDS_TARGETS rtt_roscomm  rtt_std_msgs
 rtt_geometry_msgs
 rtt_sensor_msgs
 rtt_actionlib_msgs
 rtt_nav_msgs

)
