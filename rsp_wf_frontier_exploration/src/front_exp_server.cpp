#include "rsp_wf_frontier_exploration/front_exp_server.hpp"

#include "rsp_wf_frontier_exploration/wavefront_frontier_detection.hpp"

FrontierExplorer::FrontierExplorer(ros::NodeHandle& nh) : _nh(nh) {
    _server = nh.advertiseService("get_frontier_exploration_goal",
                                  &FrontierExplorer::getGoal, this);
}

bool FrontierExplorer::getGoal(
    rsp_turtlebot3_msgs::get_frontier_exploration_goal::Request& req,
    rsp_turtlebot3_msgs::get_frontier_exploration_goal::Response& res) {
    turtle_pose current_pose = req.curr_pose;
    float current_x = current_pose.x;
    float current_y = current_pose.y;
    nav_msgs::OccupancyGrid map = req.map;
    float resolution = map.info.resolution;

    float origin_x = map.info.origin.position.x;
    float origin_y = map.info.origin.position.y;

    // float min_dis = std::numeric_limits<float>::infinity();
    int grid_x = (current_x - origin_x) / resolution;
    int grid_y = (current_y - origin_y) / resolution;
    std::vector<std::vector<int> > frontiers =
        wfd(map, map.info.height, map.info.width,
            grid_x + (grid_y * map.info.width));

    res.goal_pose.x = current_x;
    res.goal_pose.y = current_y;

    turtle_pose nearest_pose;
    turtle_pose local_furthest_pose;
    float local_radius = 1.0;

    float local_max_dis = 0.0;
    float min_dis = std::numeric_limits<float>::infinity();

    for (int i = 0; i < frontiers.size(); i++) {
        for (int j = 0; j < frontiers[i].size(); j++) {
            float x =
                ((frontiers[i][j] % map.info.width)) * resolution + origin_x;
            float y =
                ((frontiers[i][j] / map.info.width)) * resolution + origin_y;
            float diff_x = x - current_x;
            float diff_y = y - current_y;

            float dis = sqrt(diff_x * diff_x + diff_y * diff_y);
            if (dis < min_dis) {
                // return the closest frontier points
                min_dis = dis;
                nearest_pose.x = x;
                nearest_pose.y = y;
            }

            if (dis < local_radius && dis > local_max_dis) {
                local_max_dis = dis;
                local_furthest_pose.x = x;
                local_furthest_pose.y = y;
            }
        }
    }

    res.goal_pose = min_dis < local_radius ? local_furthest_pose : nearest_pose;

    // no yaw change
    res.goal_pose.yaw = current_pose.yaw;
    return true;
}