#include "rsp_loop_detection/rsp_loop_detector.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rsp_loop_detection");
    ros::NodeHandle nh("~");

    LoopDetector detector(nh);
    ros::spin();

    return 0;
}