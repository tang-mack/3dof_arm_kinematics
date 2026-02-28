#pragma once

#include "planar_arm_kinematics/core/types.h"

namespace planar_arm {

// Custom Math Backend (analytical solver)
class CustomSolver {
public:
    CustomSolver(double l1 = 0.3, double l2 = 0.3, double l3 = 0.1);

    Pose_XY_Yaw forward_kinematics(const JointAnglesRad& joint_angles) const;
    JointAnglesRad inverse_kinematics(const Pose_XY_Yaw& target) const;

private:
    double l1_{0.0};
    double l2_{0.0};
    double l3_{0.0};
};

} // namespace planar_arm