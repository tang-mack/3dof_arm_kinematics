#pragma once

#include "planar_arm_kinematics/core/Types.h"
#include "planar_arm_kinematics/core/RobotModel.h"

#include "planar_arm_kinematics/core/YamlReader.h"
#include <vector>
#include <string>
#include <optional>

namespace planar_arm {

struct AnalyticalSolverConfig {
    std::optional<std::string> urdf_filepath; // optional for semantic clarity: if it works without it, convey that
    std::string link_length_source; // Where to pull link lengths from: "urdf" or "yaml"
    std::vector<double> link_lengths;
    bool use_lookup_table_speedup;
    std::vector<std::vector<double>> joint_limits;

};

// Custom FK/IK Math Backend (analytical solver)
class AnalyticalSolver {
public:
    /// @brief Typical usage: Fill out config struct using yaml file
    explicit AnalyticalSolver(const std::string& yaml_filepath);

    /// @ brief Do not use yaml to populate config struct: inject config struct directly
    explicit AnalyticalSolver(const AnalyticalSolverConfig& config, const RobotModel& model);

    Pose_XY_Yaw forward_kinematics(const JointAnglesRad& joint_angles) const;
    bool inverse_kinematics(const Pose_XY_Yaw& ee_target, JointAnglesRad& q_solution, const JointAnglesRad& q_guess, IKStatus& status) const;

private:
    /// @brief Wraps angle to [-pi to pi]
    /// @param angle Angle to wrap in [rad]
    double wrap_to_pi(const double angle) const;

    // Configuration members
    YamlReader yaml_reader_;
    AnalyticalSolverConfig config_;

    // Data members
    RobotModel model_;


};

} // namespace planar_arm