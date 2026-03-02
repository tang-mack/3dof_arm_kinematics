#include "planar_arm_kinematics/core/kinematics.h"
#include <iostream>

namespace planar_arm {

// Implementations
template <typename SolverBackend>
Kinematics<SolverBackend>::Kinematics(SolverBackend solver) : solver_(solver) {}

template <typename SolverBackend>
Pose_XY_Yaw Kinematics<SolverBackend>::compute_fk(const JointAnglesRad& joint_angles_rad) const {
    // Pass straight through to the backend
    return solver_.forward_kinematics(joint_angles_rad);
    // return Pose_XY_Yaw{0.0, 0.0, 0.0};
}

template <typename SolverBackend>
JointAnglesRad Kinematics<SolverBackend>::compute_ik(const Pose_XY_Yaw& target, const double guess_elbow_joint) const {
    // Pass straight through to the backend
    return solver_.inverse_kinematics(target, guess_elbow_joint);
    // return JointAnglesRad{-99.0, -99.0, -99.0};
}

// ==============================================================================
// EXPLICIT TEMPLATE INSTANTIATIONS
// Templates normally require implementations in the header. By explicitly 
// instantiating the types here, we pre-compile the machine code for these 
// specific solvers, allowing us to keep the implementation in this .cpp 
// file without causing "undefined reference" linker errors.
// ==============================================================================
template class Kinematics<CustomSolver>;

#ifdef USE_PINOCCHIO_MODE
template class Kinematics<PinocchioSolver>;
#endif

} // namespace planar_arm