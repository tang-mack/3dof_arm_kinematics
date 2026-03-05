#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <memory>
#include "planar_arm_kinematics/core/KinematicSolver.h"

namespace planar_arm {

class FkPublisherNode : public rclcpp::Node {
public:
    FkPublisherNode();

private:
    void timer_callback();

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<KinematicSolver> solver_; // Possible backends for KinematicSolver: AnalyticalSolver, PinocchioSolver
};

} // namespace planar_arm