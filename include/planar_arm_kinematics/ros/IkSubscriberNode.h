#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <memory>
#include "planar_arm_kinematics/core/KinematicSolver.h"

namespace planar_arm {

class IkSubscriberNode : public rclcpp::Node {
public:
    IkSubscriberNode();

private:
    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscription_;

    std::unique_ptr<KinematicSolver> solver_;

    JointAnglesRad previous_joints_;
};

} // namespace planar_arm