#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <memory>
#include "3dof_arm_kinematics/core/KinematicSolver.h"

namespace arm_3dof {

/// @brief Solves IK repeatedly, prints to screen
class IkSubscriberNode : public rclcpp::Node {
public:
    // Setup launch file parameters, pick solver backend, create subscription
    IkSubscriberNode();

private:
    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr subscription_;

    std::unique_ptr<KinematicSolver> solver_;

    JointAnglesRad previous_joints_;
};

} // namespace arm_3dof