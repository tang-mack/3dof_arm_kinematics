#pragma once

#include "planar_arm_kinematics/core/Types.h"
#include "planar_arm_kinematics/core/RobotModel.h"

#include "planar_arm_kinematics/core/YamlReader.h"
#include <vector>
#include <string>

namespace planar_arm {

struct AnalyticalSolverConfig {
    std::string urdf_filepath;
    std::string parse_lengths_from; // Where to get the link lengths from: "urdf" or "yaml"
    std::vector<double> link_lengths;
    bool use_lookup_table_speedup;
    std::vector<std::vector<double>> joint_limits;

};

// Custom FK/IK Math Backend (analytical solver)
class AnalyticalSolver {
public:
    /// @brief Yaml-based constructor for typical usage
    explicit AnalyticalSolver(const std::string& yaml_filepath, const RobotModel& robot_model);

    /// @ brief Struct-based constructor mainly for Unit Tests
    explicit AnalyticalSolver(const AnalyticalSolverConfig& config, const RobotModel& robot_model);


    Pose_XY_Yaw forward_kinematics(const JointAnglesRad& joint_angles) const;
    JointAnglesRad inverse_kinematics(const Pose_XY_Yaw& end_effector_target, const double guess_elbow_joint) const;

private:

    /// @brief Wraps angle to [-pi to pi]
    /// @param angle Angle to wrap in [rad]
    double wrap_to_pi(const double angle) const;

    RobotModel robot_model_;

    double l1_;
    double l2_;
    double l3_;

    // Configuration members
    YamlReader yaml_reader_;
    AnalyticalSolverConfig config_;

};

} // namespace planar_arm