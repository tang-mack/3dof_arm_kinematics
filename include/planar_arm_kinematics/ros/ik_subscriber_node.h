#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "planar_arm_kinematics/core/kinematics.h"

namespace planar_arm {

class IkSubscriberNode : public rclcpp::Node {
public:
    IkSubscriberNode();

private:
    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscription_;
    ActiveKinematics kinematics_;
};

} // namespace planar_arm