#include "rsp_loop_detection/rsp_loop_detector.hpp"

LoopDetector::LoopDetector(ros::NodeHandle& nh) : _nh(nh) {
    // setup serviceds
    std::string add_scan_service;
    std::string loop_detection_service;
    _nh.param<std::string>("add_scan_service", add_scan_service, "add_scan");
    _nh.param<std::string>("detect_loop_service", add_scan_service, "add_scan");

    _add_scan_server =
        _nh.advertiseService(add_scan_service, &LoopDetector::add_scan, this);
    _loop_detection_server = _nh.advertiseService(
        loop_detection_service, &LoopDetector::detect_loop, this);

    // parameters for loop detector
    _nh.param<double>("scan_distance", _scan_distance, 0.1);
    _nh.param<double>("similarity_threshold", _similarity_threshold, 1.0);
    _nh.param<int>("yaw_search_window", _yaw_search_window, 30);
    _nh.param<int>("ignore_recent_scan", _ignore_recent_scan, 10);
    _nh.param<int>("num_ranges", _num_ranges, 360);
    _nh.param<int>("min_num_valid_ranges", _min_num_valid_ranges, 180);

    _nh.param<double>("max_range", _max_range, 3.0);
}
LoopDetector::~LoopDetector() {}

bool LoopDetector::add_scan(add_scan_req& req, add_scan_res& res) {
    if (_scans.empty()) {
        _scans.emplace_back(req);
    } else {
        const auto& last_scan = _scans.back();
        const double dx = req.pose.x - last_scan.pose.x;
        const double dy = req.pose.y - last_scan.pose.y;
        const double dis = sqrt(dx * dx + dy * dy);
        if (dis > _scan_distance) {
            _scans.emplace_back(req);
        }
    }
    res.num_scans = _scans.size();
    return true;
}

bool LoopDetector::detect_loop(rsp_turtlebot3_msgs::detect_loopRequest& req,
                               rsp_turtlebot3_msgs::detect_loopResponse& res) {
    double best_similarity = std::numeric_limits<double>::max();
    for (int i = 0; i < _scans.size() - _ignore_recent_scan; ++i) {
        const auto& cand_scan = _scans[i];
        const int yaw_offset =
            static_cast<int>(cand_scan.pose.yaw - req.pose.yaw);
        for (int query_start_idx = yaw_offset - _yaw_search_window / 2;
             query_start_idx <= yaw_offset + _yaw_search_window / 2;
             ++query_start_idx) {
            double similarity = 0.0;
            int num_valid_range = 0;
            for (int cand_idx = 0; cand_idx < _num_ranges; ++cand_idx) {
                int query_idx = query_start_idx + cand_idx;
                query_idx = query_idx >= _num_ranges ? query_idx - _num_ranges
                                                     : query_idx;
                query_idx = query_idx < 0 ? query_idx + _num_ranges : query_idx;

                double q_range = req.scan.ranges[query_idx];
                double c_range = cand_scan.scan.ranges[cand_idx];
                if (q_range < _max_range && c_range < _max_range) {
                    ++num_valid_range;
                    const double diff = q_range - c_range;
                    similarity += diff * diff;
                }
            }
            if (num_valid_range > _min_num_valid_ranges) {
                similarity = sqrt(similarity);
                best_similarity =
                    similarity < best_similarity ? similarity : best_similarity;
            }
        }
    }
    res.detected = best_similarity < _similarity_threshold;
    return true;
}