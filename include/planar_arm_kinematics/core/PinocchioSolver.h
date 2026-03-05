#pragma once

#include "planar_arm_kinematics/core/KinematicSolver.h"

#include "planar_arm_kinematics/core/Types.h"
#include <string>

#include <pinocchio/fwd.hpp> // Pinocchio forward declarations (required before standard pinocchio includes)
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace planar_arm {

struct PinocchioSolverConfig {
    // Parameters from Yaml file
    std::string urdf_filepath;
    bool use_lookup_table_speedup;
};


class PinocchioSolver : public KinematicSolver {
public:
    explicit PinocchioSolver(const std::string& yaml_filepath);
    explicit PinocchioSolver(const PinocchioSolverConfig& config);
    
    Pose_XY_Yaw forward_kinematics(const JointAnglesRad& joints) const override;
    bool inverse_kinematics(const Pose_XY_Yaw& ee_target, JointAnglesRad& q_solution, const JointAnglesRad& q_guess, IKStatus& status) const override;

private:
    void load_urdf(const std::string& absolute_urdf_path);
    void calculate_maximum_reach_from_urdf();

    PinocchioSolverConfig config_;

    // Pinocchio specific members
    pinocchio::Model model_;
    mutable pinocchio::Data data_; // Mutable because Pinocchio writes intermediate math to this struct
    int ee_frame_id_; // Cached ID for end effector

    double max_reach_{0.0}; // [m] robot arm fully extended
};

} // namespace planar_arm