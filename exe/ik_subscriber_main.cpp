#include <rclcpp/rclcpp.hpp>
#include "3dof_arm_kinematics/ros/IkSubscriberNode.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<arm_3dof::IkSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}