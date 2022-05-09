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
    ros::NodeHandle _nh;
    ros::ServiceServer _add_scan_server;
    ros::ServiceServer _loop_detection_server;

    double _scan_distance;
    double _similarity_threshold;
    int _yaw_search_window;
    int _ignore_recent_scan;
    int _num_ranges;
    int _min_num_valid_ranges;
    double _max_range;

    std::vector<add_scan_req> _scans;
};