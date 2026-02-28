#include <rclcpp/rclcpp.hpp>
#include "planar_arm_kinematics/ros/fk_publisher_node.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<planar_arm::FkPublisherNode>());
    rclcpp::shutdown();
    return 0;
}