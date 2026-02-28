#pragma once

#include "planar_arm_kinematics/core/types.h"
#include <string>

namespace planar_arm {

#ifdef USE_PINOCCHIO_MODE
class PinocchioSolver {
public:
    PinocchioSolver(const std::string& urdf_path);
    
    Pose_XY_Yaw forward_kinematics(const JointAnglesRad& joints) const;
    JointAnglesRad inverse_kinematics(const Pose_XY_Yaw& target) const;
};
#endif

} // namespace planar_arm