#include <cmath>
#include "planar_arm_kinematics/core/AnalyticalSolver.h"

#include <algorithm> // for std::clamp
#include <iostream>

namespace planar_arm {

// Constructor 1: Yaml-based constructor for typical usage
AnalyticalSolver::AnalyticalSolver(const std::string& yaml_filepath, const RobotModel& robot_model)
    : robot_model_(robot_model),
    l1_(robot_model.L1),
    l2_(robot_model.L2),
    l3_(robot_model.L3) {
        // load() will throw if yaml does not exist
        yaml_reader_.load(yaml_filepath);

        // 1. Parse Global Yaml Parameters (Stuff at "root" level of yaml file)
        config_.urdf_filepath = yaml_reader_.get_global<std::string>("urdf_filepath");

        // 2. Parse Local Yaml Parameters (ie. nested under AnalyticalSolver within the yaml file)
        config_.parse_lengths_from = yaml_reader_.get_local<std::string>("AnalyticalSolver", "parse_lengths_from_urdf_vs_yaml");
        config_.link_lengths = yaml_reader_.get_local<std::vector<double>>("AnalyticalSolver", "link_lengths");
        config_.use_lookup_table_speedup = yaml_reader_.get_local<bool>("AnalyticalSolver", "use_lookup_table_speedup");

        // Parse Joint Limits from Yaml
        std::vector<std::vector<double>> default_limits = {{-178.0, 178.0}, {-178.0, 178.0}, {-178.0, 178.0}};
        config_.joint_limits = yaml_reader_.get_local<std::vector<std::vector<double>>>("AnalyticalSolver", "joint_limits");

        // YAML overrides URDF link lengths logic
        if (config_.parse_lengths_from == "yaml") {
            if (config_.link_lengths.size() == 3) {
                l1_ = config_.link_lengths[0];
                l2_ = config_.link_lengths[1];
                l3_ = config_.link_lengths[2];
            } else {
                throw std::runtime_error("[AnalyticalSolver] YAML link_lengths must contain exactly 3 values.");
            }
        }
        
        std::cout << "[AnalyticalSolver] Configuration loaded successfully." << std::endl;

    }

    // Constructor 2: Struct-based constructor mainly for Unit Tests
    AnalyticalSolver::AnalyticalSolver(const AnalyticalSolverConfig& config, const RobotModel& robot_model)
        : robot_model_(robot_model),
          l1_(robot_model.L1),
          l2_(robot_model.L2),
          l3_(robot_model.L3),
          config_(config)
    {
        // YAML overrides URDF link lengths logic
        if (config_.parse_lengths_from == "yaml") {
            if (config_.link_lengths.size() == 3) {
                l1_ = config_.link_lengths[0];
                l2_ = config_.link_lengths[1];
                l3_ = config_.link_lengths[2];
            } else {
                throw std::runtime_error("[AnalyticalSolver] Config link_lengths must contain exactly 3 values.");
            }
        }
    }

Pose_XY_Yaw AnalyticalSolver::forward_kinematics(const JointAnglesRad& joint_angles) const {

    double q1 = joint_angles.at(0);
    double q2 = joint_angles.at(1);
    double q3 = joint_angles.at(2);

    double x = l1_ * std::cos(q1) + l2_ * std::cos(q1 + q2) + l3_ * std::cos(q1 + q2 + q3);
    double y = l1_ * std::sin(q1) + l2_ * std::sin(q1 + q2) + l3_ * std::sin(q1 + q2 + q3);
    double theta = q1 + q2 + q3;

    Pose_XY_Yaw pose{x, y, theta};

    return pose;
}

double AnalyticalSolver::wrap_to_pi(const double angle) const {
    return std::atan2(std::sin(angle), std::cos(angle));
}

JointAnglesRad AnalyticalSolver::inverse_kinematics(const Pose_XY_Yaw& end_effector_target, const double guess_elbow_joint) const {
    // std::cout << "[AnalyticalSolver] inverse_kinematics called" << std::endl;

    double x = end_effector_target.x;
    double y = end_effector_target.y;
    double yaw = end_effector_target.yaw;

    // Trick: solve wrist first
    double x_wrist = x - l3_ * std::cos(yaw);
    double y_wrist = y - l3_ * std::sin(yaw);

    double arccos_input = (x_wrist*x_wrist + y_wrist*y_wrist - l1_*l1_ - l2_*l2_) / (2 * l1_ * l2_);
    arccos_input = std::clamp(arccos_input, -1.0, 1.0); // If the input to arccos is beyond -1.0 to 1.0, std::acos will return NaN, possibly causing issues.

    // arc cosine returns two values. std::acos returns the principal value (0 to pi). The second solution is given by -std::acos
    double theta_2_sol1 = std::acos(arccos_input); // Elbow down
    double theta_2_sol2 = -std::acos(arccos_input); // Elbow up

    double theta_2{0.0};
    if (std::abs(wrap_to_pi(theta_2_sol1 - guess_elbow_joint)) < std::abs(wrap_to_pi(theta_2_sol2 - guess_elbow_joint))) {
        theta_2 = theta_2_sol1;
    }
    else {
        theta_2 = theta_2_sol2;
    }

    // Solve for theta_1 using theta_2
    double theta_1 = std::atan2(y_wrist, x_wrist) - std::atan2(l2_*std::sin(theta_2), l1_+l2_*std::cos(theta_2));

    // Solve for theta_3
    double theta_3 = yaw - theta_1 - theta_2;

    // Strictly normalize all angles between [-pi, pi] just in case
    return JointAnglesRad{wrap_to_pi(theta_1), wrap_to_pi(theta_2), wrap_to_pi(theta_3)};
}



} // namespace planar_arm