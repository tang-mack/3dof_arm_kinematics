#pragma once

#include "planar_arm_kinematics/core/Types.h"
#include <string>

// Pinocchio forward declarations (required before standard pinocchio includes)
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

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
    bool inverse_kinematics(const Pose_XY_Yaw& ee_target, JointAnglesRad& q_solution, const JointAnglesRad& q_guess, IKStatus& status) const;

private:
    void load_urdf(const std::string& absolute_urdf_path);
    void calculate_maximum_reach_from_urdf();

    PinocchioSolverConfig config_;

    // Pinocchio specific members
    pinocchio::Model model_;
    mutable pinocchio::Data data_; // Mutable because Pinocchio writes intermediate math to this struct
    int ee_frame_id_; // Cached ID for end effector

    double max_reach_{0.0}; // [m], for checking IK target out-of-reach
};

} // namespace planar_arm