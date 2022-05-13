#include <ros/ros.h>

#include "rsp_turtlebot3_msgs/add_scan_to_loop_detection_database.h"
#include "rsp_turtlebot3_msgs/detect_loop.h"

typedef rsp_turtlebot3_msgs::add_scan_to_loop_detection_databaseRequest
    add_scan_req;
typedef rsp_turtlebot3_msgs::add_scan_to_loop_detection_databaseResponse
    add_scan_res;

class LoopDetector {
   public:
    LoopDetector(ros::NodeHandle& nh);
    ~LoopDetector();

    bool add_scan(add_scan_req& req, add_scan_res& res);

    bool detect_loop(rsp_turtlebot3_msgs::detect_loopRequest& req,
                     rsp_turtlebot3_msgs::detect_loopResponse& res);

   private:
    ros::NodeHandle nh_;
    ros::ServiceServer add_scan_server_;
    ros::ServiceServer loop_detection_server_;

    double scan_distance_;
    double similarity_threshold_;
    int yaw_search_window_;
    int ignore_recent_scan_;
    int num_ranges_;
    int min_num_valid_ranges_;
    double max_range_;

    std::vector<add_scan_req> scans_;
};