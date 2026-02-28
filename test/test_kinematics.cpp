#include <gtest/gtest.h>
#include "planar_arm_kinematics/core/kinematics.h"
#include "planar_arm_kinematics/core/custom_solver.h"

using namespace planar_arm;

// Test 1: Forward Kinematics at Zero Configuration
TEST(KinematicsTest, ForwardKinematicsZeroAngles) {
    // Instantiate with default lengths: L1=0.3, L2=0.3, L3=0.1
    CustomSolver solver(0.3, 0.3, 0.1);
    
    // Set all joint angles to 0.0 radians
    JointAnglesRad joints = {0.0, 0.0, 0.0};
    
    Pose_XY_Yaw pose = solver.forward_kinematics(joints);
    
    // If the arm is fully extended on the X-axis, X should be the sum of all link lengths (0.7m)
    EXPECT_DOUBLE_EQ(pose.x, 0.7);
    EXPECT_DOUBLE_EQ(pose.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.yaw, 0.0);
}

// Test 2: Inverse Kinematics from the Zero Configuration
TEST(KinematicsTest, InverseKinematicsZeroConfiguration) {
    CustomSolver solver(0.3, 0.3, 0.1);
    
    // Target the fully extended position
    Pose_XY_Yaw target{0.7, 0.0, 0.0};
    
    JointAnglesRad joints = solver.inverse_kinematics(target);
    
    // The angles required to reach this should all be 0.0
    EXPECT_DOUBLE_EQ(joints[0], 0.0);
    EXPECT_DOUBLE_EQ(joints[1], 0.0);
    EXPECT_DOUBLE_EQ(joints[2], 0.0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}