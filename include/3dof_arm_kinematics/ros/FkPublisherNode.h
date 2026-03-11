#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <memory>
#include "3dof_arm_kinematics/core/KinematicSolver.h"

namespace arm_3dof {

/// @brief Intake joint angles, run FK, publish end-effector to topic
class FkPublisherNode : public rclcpp::Node {
public:
    /// @brief Intake launch file parameters, pick solver backend, create publisher
    FkPublisherNode();

private:
    void timer_callback();

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<KinematicSolver> solver_; // Possible backends for KinematicSolver: AnalyticalSolver, PinocchioSolver
};

} // namespace arm_3dof