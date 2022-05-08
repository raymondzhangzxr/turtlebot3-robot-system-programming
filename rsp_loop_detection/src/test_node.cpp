#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

void scanCallback(const sensor_msgs::LaserScanConstPtr& scan){
    std::cout << scan->ranges.size() << std::endl;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "loop_detection");
    ros::NodeHandle nh;

    ros::Subscriber scan_sub = nh.subscribe("scan", 1000, scanCallback);

    ros::spin();

    return 0;
}