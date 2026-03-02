#pragma once

#include "planar_arm_kinematics/core/Types.h"
#include "planar_arm_kinematics/core/AnalyticalSolver.h"

#ifdef USE_PINOCCHIO_MODE
#include "planar_arm_kinematics/core/PinocchioSolver.h"
#endif

namespace planar_arm {


template <typename SolverBackend>
class Kinematics {
public:
    explicit Kinematics(SolverBackend solver);
    Kinematics() = delete;

    Pose_XY_Yaw compute_fk(const JointAnglesRad& joint_angles_rad) const;
    JointAnglesRad compute_ik(const Pose_XY_Yaw& target, const double guess_elbow_joint) const;

private:
    SolverBackend solver_;
    JointAnglesRad previous_angles_; // Store previous joint angles

};

// Compile-Time Selection Alias (pick between Solver Backends)
#ifdef USE_PINOCCHIO_MODE
    using ActiveKinematics = Kinematics<PinocchioSolver>;
#else
    using ActiveKinematics = Kinematics<AnalyticalSolver>;
#endif

} // namespace planar_arm