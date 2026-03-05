#pragma once

#include "planar_arm_kinematics/core/KinematicSolver.h"

#include "planar_arm_kinematics/core/Types.h"
#include "planar_arm_kinematics/core/RobotModel.h"

#include "planar_arm_kinematics/core/YamlReader.h"
#include <vector>
#include <string>
#include <optional>

namespace planar_arm {

struct AnalyticalSolverConfig {
    std::optional<std::string> urdf_filepath; // optional for semantic clarity: AnalyticalSolver works from yaml, or from urdf.
    std::string link_length_source; // Where to pull link lengths from: "urdf" or "yaml"
    std::vector<double> link_lengths;
    bool use_lookup_table_speedup;
    std::vector<std::vector<double>> joint_limits; // [proximal ... distal] [lower_lim upper_lim]. Ex: vec.at(shoulder index is 0).at(0) = shoulder limit -178 degrees

};


class AnalyticalSolver : public KinematicSolver {
public:
    // Typical usage: Fill out config struct using yaml file.
    explicit AnalyticalSolver(const std::string& yaml_filepath);

    // Do not use yaml to populate config struct: inject config struct directly
    explicit AnalyticalSolver(const AnalyticalSolverConfig& config, const RobotModel& model);

    Pose_XY_Yaw forward_kinematics(const JointAnglesRad& joint_angles) const override;
    bool inverse_kinematics(const Pose_XY_Yaw& ee_target, JointAnglesRad& q_solution, const JointAnglesRad& q_guess, IKStatus& status) const override;

private:
    /// @brief Wraps angle to [-pi to pi]
    /// @param angle Angle to wrap in [rad]
    double wrap_to_pi(const double angle) const;

    void load_config_from_yaml(const std::string& yaml_filepath);

    void initialize_robot_model(const std::string& yaml_filepath);

    YamlReader yaml_reader_;
    AnalyticalSolverConfig config_;
    RobotModel model_;

};

} // namespace planar_arm