#pragma once

#include "planar_arm_kinematics/core/Types.h"
#include "planar_arm_kinematics/core/AnalyticalSolver.h"
#include "planar_arm_kinematics/core/PinocchioSolver.h"

namespace planar_arm {

template <typename SolverBackend>
class Kinematics {
public:
    explicit Kinematics(SolverBackend solver) : solver_(solver) {};

    Kinematics() = delete;

    Pose_XY_Yaw compute_fk(const JointAnglesRad& joint_angles_rad) const {
        // Pass straight through to the backend to solve
        return solver_.forward_kinematics(joint_angles_rad);
    }

    JointAnglesRad compute_ik(const Pose_XY_Yaw& target, const double guess_elbow_joint) const {
        // Pass straight through to the backend  to solve
        return solver_.inverse_kinematics(target, guess_elbow_joint);
    }

private:
    SolverBackend solver_;
    JointAnglesRad previous_angles_; // Store previous joint angles

};


} // namespace planar_arm