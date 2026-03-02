#include "planar_arm_kinematics/core/PinocchioSolver.h"

namespace planar_arm {

#ifdef USE_PINOCCHIO_MODE
PinocchioSolver::PinocchioSolver(const std::string& urdf_path) {
    // Pinocchio initialization
}

Pose_XY_Yaw PinocchioSolver::forward_kinematics(const JointAnglesRad& joints) const {
    return Pose_XY_Yaw{0.0, 0.0, 0.0};
}

JointAnglesRad PinocchioSolver::inverse_kinematics(const Pose_XY_Yaw& target) const {
    return JointAnglesRad{0.0, 0.0, 0.0};
}
#endif

} // namespace planar_arm