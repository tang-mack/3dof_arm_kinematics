#pragma once

#include "3dof_arm_kinematics/core/Types.h"

namespace arm_3dof {

/// @brief Base class API for the following Solver Backends: AnalyticalSolver, PinocchioSolver
class KinematicSolver {
public:
    virtual ~KinematicSolver() = default; // mandatory in base classes to prevent memory leaks

    // Calculate end-effector pose given joint angles
    virtual Pose_XY_Yaw forward_kinematics(const JointAnglesRad& joints) const = 0;

    // Returns True if valid solution found, false otherwise. q_solution passes through unchanged if invalid solve. Error-handling with status.
    virtual bool inverse_kinematics(const Pose_XY_Yaw& ee_target, JointAnglesRad& q_solution, const JointAnglesRad& q_guess, IKStatus& status) const = 0;

};

} // namespace arm_3dof