#ifndef PATH_HPP
#define PATH_HPP

#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>

class Path {
public:
    Path() = default;

    // 添加航点
    void add_waypoint(const geometry_msgs::msg::PoseStamped& waypoint) {
        waypoints.push_back(waypoint);
    }

    // 获取下一个航点
    bool get_next_waypoint(geometry_msgs::msg::PoseStamped& waypoint) {
        if (current_index < waypoints_.size()) {
            waypoint = waypoints[current_index++];
            return true;
        }
        return false; // 所有航点已发送
    }

    void reset() { current_index = 0; } // 重置索引

private:
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    size_t current_index = 0;
};

#endif
