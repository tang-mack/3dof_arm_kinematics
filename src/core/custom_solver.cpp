#include <cmath>
#include "planar_arm_kinematics/core/custom_solver.h"
#include <iostream>


namespace planar_arm {

// Custom Math Backend Implementation
CustomSolver::CustomSolver(double l1, double l2, double l3)
    : l1_(l1), l2_(l2), l3_(l3) {}

Pose_XY_Yaw CustomSolver::forward_kinematics(const JointAnglesRad& joint_angles) const {

    double q1 = joint_angles.at(0);
    double q2 = joint_angles.at(1);
    double q3 = joint_angles.at(2);

    double x = l1_ * std::cos(q1) + l2_ * std::cos(q1 + q2) + l3_ * std::cos(q1 + q2 + q3);
    double y = l1_ * std::sin(q1) + l2_ * std::sin(q1 + q2) + l3_ * std::sin(q1 + q2 + q3);
    double theta = q1 + q2 + q3;

    Pose_XY_Yaw pose{x, y, theta};

    return pose;
}

JointAnglesRad CustomSolver::inverse_kinematics(const Pose_XY_Yaw& target) const {
    std::cout << "[CustomSolver] inverse_kinematics called" << std::endl;

    // Trick: solve wrist first


    JointAnglesRad joints{0.0, 0.0, 0.0};
    return joints;
}



} // namespace planar_arm