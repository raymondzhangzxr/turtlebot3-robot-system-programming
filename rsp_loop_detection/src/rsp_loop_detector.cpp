#include "rsp_loop_detection/rsp_loop_detector.hpp"

#include <iomanip>

LoopDetector::LoopDetector(ros::NodeHandle& nh) : nh_(nh) {
    // setup serviceds
    std::string add_scan_service;
    std::string loop_detection_service;
    nh_.param<std::string>("add_scan_service", add_scan_service, "add_scan");
    nh_.param<std::string>("detect_loop_service", loop_detection_service,
                           "detect_loop");

    add_scan_server_ =
        nh_.advertiseService(add_scan_service, &LoopDetector::add_scan, this);
    loop_detection_server_ = nh_.advertiseService(
        loop_detection_service, &LoopDetector::detect_loop, this);

    // parameters for loop detector
    nh_.param<double>("scan_distance", scan_distance_, 0.1);
    nh_.param<double>("similarity_threshold", similarity_threshold_, 1.0);
    nh_.param<int>("yaw_search_window", yaw_search_window_, 30);
    nh_.param<int>("ignore_recent_scan", ignore_recent_scan_, 10);
    nh_.param<int>("num_ranges", num_ranges_, 360);
    nh_.param<int>("min_num_valid_ranges", min_num_valid_ranges_, 180);

    nh_.param<double>("max_range", max_range_, 3.0);
    ROS_INFO_STREAM("Loop detection " << add_scan_service << " and "
                                      << loop_detection_service << " are ready");
}
LoopDetector::~LoopDetector() {}

bool LoopDetector::add_scan(add_scan_req& req, add_scan_res& res) {
    if (scans_.empty()) {
        scans_.emplace_back(req);
    } else {
        const auto& last_scan = scans_.back();
        const double dx = req.pose.x - last_scan.pose.x;
        const double dy = req.pose.y - last_scan.pose.y;
        const double dis = sqrt(dx * dx + dy * dy);
        if (dis > scan_distance_) {
            scans_.emplace_back(req);
            // ROS_INFO_STREAM(
            //     "Added new scan to loop detection database! number of scans: "
            //     << scans_.size());
        }
    }
    res.num_scans = scans_.size();
    return true;
}

bool LoopDetector::detect_loop(rsp_turtlebot3_msgs::detect_loopRequest& req,
                               rsp_turtlebot3_msgs::detect_loopResponse& res) {
    double best_similarity = std::numeric_limits<double>::max();
    int match_scan_idx = static_cast<int>(scans_.size()) - 1;
    int match_yaw_offset = 0;
    ROS_INFO_STREAM("Number of Scans: " << scans_.size());
    for (int i = 0; i < static_cast<int>(scans_.size()) - ignore_recent_scan_;
         ++i) {
        const auto& cand_scan = scans_[i];
        const int yaw_offset = static_cast<int>(
            (cand_scan.pose.yaw - req.pose.yaw) / M_PI * 180.0);

        const double q_origin_x = req.pose.x - cand_scan.pose.x;
        const double q_origin_y = req.pose.y - cand_scan.pose.y;

        for (int query_start_idx = yaw_offset - yaw_search_window_ / 2;
             query_start_idx <= yaw_offset + yaw_search_window_ / 2;
             ++query_start_idx) {
            double similarity = 0.0;
            int num_valid_range = 0;
            for (int cand_idx = 0; cand_idx < num_ranges_; ++cand_idx) {
                int query_idx = query_start_idx + cand_idx;
                query_idx = query_idx >= num_ranges_ ? query_idx - num_ranges_
                                                     : query_idx;
                query_idx = query_idx < 0 ? query_idx + num_ranges_ : query_idx;

                double q_range = req.scan.ranges[query_idx];
                double c_range = cand_scan.scan.ranges[cand_idx];
                if (q_range < max_range_ && c_range < max_range_) {
                    ++num_valid_range;
                    double curr_yaw =
                        cand_idx / num_ranges_ * cand_scan.scan.angle_increment;
                    double corrected_x = q_origin_x + q_range * cos(curr_yaw);
                    double corrected_y = q_origin_y + q_range * sin(curr_yaw);
                    double corrected_q_range = sqrt(corrected_x * corrected_x +
                                                    corrected_y * corrected_y);
                    const double diff = corrected_q_range - c_range;
                    similarity += diff * diff;
                }
            }
            if (num_valid_range > min_num_valid_ranges_) {
                similarity = sqrt(similarity);
                if (similarity < best_similarity) {
                    best_similarity = similarity;
                    match_scan_idx = i;
                    match_yaw_offset = query_start_idx;
                }
            }
        }
    }
    const auto& match_scan = scans_[match_scan_idx];
    ROS_INFO_STREAM("best similarity: " << best_similarity);
    ROS_INFO_STREAM("query scan [x y theta]: "
                    << req.pose.x << " " << req.pose.y << " " << req.pose.yaw);
    ROS_INFO_STREAM("matched scan [x y theta]: " << match_scan.pose.x << " "
                                                 << match_scan.pose.y << " "
                                                 << match_scan.pose.yaw);
    ROS_INFO_STREAM("yaw offset(degree): " << match_yaw_offset);

    res.detected = best_similarity < similarity_threshold_;
    return true;
}