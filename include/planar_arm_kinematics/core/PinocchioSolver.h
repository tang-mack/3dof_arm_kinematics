#pragma once

#include "planar_arm_kinematics/core/Types.h"
#include <string>

namespace planar_arm {

struct PinocchioSolverConfig {
    // Parameters from Yaml file
    std::string urdf_filepath;
    bool use_lookup_table_speedup;

    // Other Parameters
};

// Designed for usage with Kinematics class.
// Usage Example:
//      Kinematics<PinocchioSolver> kin_;
//      kin_.compute_ik(target_pose, previous_joints)
class PinocchioSolver {
public:
    explicit PinocchioSolver(const std::string& yaml_filepath);
    explicit PinocchioSolver(const PinocchioSolverConfig& config);
    
    Pose_XY_Yaw forward_kinematics(const JointAnglesRad& joints) const;
    JointAnglesRad inverse_kinematics(const Pose_XY_Yaw& target, const double guess_elbow_joint) const;
};

} // namespace planar_arm