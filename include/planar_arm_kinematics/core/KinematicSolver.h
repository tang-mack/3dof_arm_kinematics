#pragma once

#include "planar_arm_kinematics/core/Types.h"

namespace planar_arm {

/// @brief Base class API for the following Solver Backends: AnalyticalSolver, PinocchioSolver
class KinematicSolver {
public:
    virtual ~KinematicSolver() = default; // mandatory in base classes to prevent memory leaks

    virtual Pose_XY_Yaw forward_kinematics(const JointAnglesRad& joints) const = 0;

    virtual bool inverse_kinematics(const Pose_XY_Yaw& ee_target, JointAnglesRad& q_solution, const JointAnglesRad& q_guess, IKStatus& status) const = 0;

};

} // namespace planar_arm