#include "rsp_frontier_exploration_server/front_exp_server.hpp"
#include "rsp_frontier_exploration_server/wavefront_frontier_detection.hpp"
   bool Frontier_exploration::getGoal(rsp_turtlebot3_msgs::get_frontier_exploration_goal::Request& req,
              rsp_turtlebot3_msgs::get_frontier_exploration_goal::Response& res)
    {
        turtle_pose current_pose = req.curr_pose;
        float current_x = current_pose.x;
        float current_y = current_pose.y;
        nav_msgs::OccupancyGrid map = req.map;
        float resolution = map.info.resolution;
		float map_x = map.info.origin.position.x / resolution;
		float map_y = map.info.origin.position.y / resolution;
		float x = 0. - map_x;
		float y = 0. - map_y;
        float min_dis = std::numeric_limits<float>::infinity();
        //<<continus frontier points>>
        std::vector<std::vector<int> > frontiers = wfd(map, map.info.height, map.info.width, x + (y * map.info.width));

		for(int i = 0; i < frontiers.size(); i++) {
			for(int j = 0; j < frontiers[i].size(); j++) {
				x = ((frontiers[i][j] % map.info.width) + map_x) * resolution;
				y = ((frontiers[i][j] / map.info.width) + map_y) * resolution;
                float dis = sqrt(pow(x - current_x, 2) + pow(y - current_y, 2) * 1.0);
				if( dis < min_dis){
                    // return the closest frontier points
                    min_dis = dis;
                    res.goal_pose.x = x;
                    res.goal_pose.y = y;
                } 
			}
		}
        
        // no yaw change
        res.goal_pose.yaw = current_pose.yaw;
        return true;

    }