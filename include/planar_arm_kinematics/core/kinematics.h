#pragma once

#include "planar_arm_kinematics/core/types.h"
#include "planar_arm_kinematics/core/custom_solver.h"

#ifdef USE_PINOCCHIO_MODE
#include "planar_arm_kinematics/core/pinocchio_solver.h"
#endif

namespace planar_arm {


template <typename SolverPolicy>
class Kinematics {
public:
    explicit Kinematics(SolverPolicy solver);

    Pose_XY_Yaw compute_fk(const JointAnglesRad& joint_angles_rad) const;
    JointAnglesRad compute_ik(const Pose_XY_Yaw& target) const;

private:
    SolverPolicy solver_;
};

// Compile-Time Selection Alias (pick between Solver Backends)
#ifdef USE_PINOCCHIO_MODE
    using ActiveKinematics = Kinematics<PinocchioSolver>;
#else
    using ActiveKinematics = Kinematics<CustomSolver>;
#endif

} // namespace planar_arm