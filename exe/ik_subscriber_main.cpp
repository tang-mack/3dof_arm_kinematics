#include <rclcpp/rclcpp.hpp>
#include "planar_arm_kinematics/ros/ik_subscriber_node.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<planar_arm::IkSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}