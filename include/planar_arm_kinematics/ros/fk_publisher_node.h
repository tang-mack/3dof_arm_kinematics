#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "planar_arm_kinematics/core/kinematics.h"

namespace planar_arm {

class FkPublisherNode : public rclcpp::Node {
public:
    FkPublisherNode();

private:
    void timer_callback();

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Instantiate our templated kinematics API
    ActiveKinematics kinematics_; // Pick between math solver backends
};

} // namespace planar_arm