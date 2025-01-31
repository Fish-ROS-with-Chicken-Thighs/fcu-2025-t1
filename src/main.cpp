#include "classes/quadcopter.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto quad_node = std::make_shared<quadcopter>();
    quad_node->quad_init();
    rclcpp::shutdown();
    return 0;
}