#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "planar_arm_kinematics/core/Kinematics.h"

namespace planar_arm {

// ==========================================================
// COMPILE-TIME SOLVER SELECTION
// Change 'AnalyticalSolver' to 'PinocchioSolver' to switch backends
using ActiveSolverBackend = AnalyticalSolver; 
// using ActiveSolverBackend = PinocchioSolver; 
// ==========================================================

class FkPublisherNode : public rclcpp::Node {
public:
    FkPublisherNode();

private:
    void timer_callback();

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Instantiate our templated kinematics API
    Kinematics<ActiveSolverBackend> kinematics_; // Pick between math solver backends
};

} // namespace planar_arm