#ifndef PATH_HPP
#define PATH_HPP

#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>

class path {
public:
    size_t current_index = 0;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    
    // 添加航点
    void add_waypoint(geometry_msgs::msg::PoseStamped& waypoint) {
        waypoints.push_back(waypoint);
    }

    // 删除航点，默认从尾部开始删除
    void remove_waypoint(size_t erase_num = SIZE_MAX) {
        if (waypoints.empty()) {
            std::cerr << "Waypoints list is empty, nothing to remove!" << std::endl;
            return;
        }

        if (erase_num == SIZE_MAX) { erase_num = waypoints.size(); } // 动态计算默认值

        if (erase_num < waypoints.size()) {
            waypoints.erase(waypoints.begin() + erase_num);
        } else {
            std::cerr << "Invalid waypoint index: " << erase_num << std::endl;
        }
    }

    // 获取下一个航点
    bool get_next_waypoint(geometry_msgs::msg::PoseStamped& waypoint) {
        if (current_index < waypoints.size()) {
            waypoint = waypoints[current_index++];
            return true;
        } else {
            current_index = 0;
            return false; // 所有航点已发送
        }
    }
};

#endif
