#include <gtest/gtest.h>
#include "planar_arm_kinematics/core/kinematics.h"
#include "planar_arm_kinematics/core/custom_solver.h"

#include <cmath>
#include <vector>

using namespace planar_arm;

// Declared before the tests
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

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
    
    JointAnglesRad joints = solver.inverse_kinematics(target, 30.0*M_PI/180.0);
    
    // The angles required to reach this should all be 0.0
    EXPECT_DOUBLE_EQ(joints[0], 0.0);
    EXPECT_DOUBLE_EQ(joints[1], 0.0);
    EXPECT_DOUBLE_EQ(joints[2], 0.0);
}

// Test 3: CAD Ground Truth Validation, Single Points (Table Format)
TEST(KinematicsTest, CadValidationPoints) {
    // Instantiate using standard SI units (Meters)
    CustomSolver cad_solver(0.3, 0.3, 0.1);

    // Struct to hold your giant table of numbers
    struct CadDataPoint {
        double t1_deg;
        double t2_deg;
        double t3_deg;
        double expected_x_m;
        double expected_y_m;
        double expected_yaw_deg;
    };

    // --- Large Table of CAD Ground Truth Point Tests ---
    std::vector<CadDataPoint> cad_points = {
        // t1 (deg), t2 (deg), t3 (deg), X (m),         Y (m),        Yaw (deg)
        { 17.0,      60.0,     -70.0,    453.631e-3,    392.209e-3,   7.0},
        { -17.0,      60.0,     -70.0,    595.39819e-3,    71.48895e-3,   -27.0}
        // Add more rows here...
    };

    // Run every row in the table through the solver
    for (size_t i = 0; i < cad_points.size(); ++i) {
        const auto& pt = cad_points[i];
        
        // Convert degrees to radians right before feeding into the solver
        JointAnglesRad joints = {deg2rad(pt.t1_deg), deg2rad(pt.t2_deg), deg2rad(pt.t3_deg)};
        Pose_XY_Yaw pose = cad_solver.forward_kinematics(joints);
        
        // We use EXPECT_NEAR with a 1e-5 tolerance (0.01mm) to account for CAD display rounding
        EXPECT_NEAR(pose.x, pt.expected_x_m, 1e-5) 
            << "FK X failed at table index " << i;
            
        EXPECT_NEAR(pose.y, pt.expected_y_m, 1e-5) 
            << "FK Y failed at table index " << i;
            
        EXPECT_NEAR(pose.yaw, deg2rad(pt.expected_yaw_deg), 1e-5) 
            << "FK Yaw failed at table index " << i;
    }
}


// Test 4: Inverse Kinematics CAD Ground Truth Validation
TEST(KinematicsTest, InverseKinematicsCadValidation) {
    // Instantiate using standard SI units (Meters)
    CustomSolver cad_solver(0.3, 0.3, 0.1);

    // Struct to hold CAD ground truth data points
    struct CadDataPoint {
        double t1_deg;
        double t2_deg;
        double t3_deg;
        double expected_x_m;
        double expected_y_m;
        double expected_yaw_deg;
    };

    // --- Large Table of CAD Ground Truth Point Tests ---
    std::vector<CadDataPoint> cad_points = {
        // t1 (deg), t2 (deg), t3 (deg), X (m),         Y (m),        Yaw (deg)
        { 17.0,      60.0,     -70.0,    453.631e-3,    392.209e-3,   7.0},
        { -17.0,      60.0,     -70.0,    595.39819e-3,    71.48895e-3,   -27.0}
        // Add more rows here...
    };

    // Run every row in the table through the IK solver
    for (size_t i = 0; i < cad_points.size(); ++i) {
        const auto& pt = cad_points[i];
        
        Pose_XY_Yaw target_pose{pt.expected_x_m, pt.expected_y_m, deg2rad(pt.expected_yaw_deg)};
        
        // Pass the expected elbow angle as the guess to ensure the solver selects the matching configuration
        double elbow_guess = deg2rad(pt.t2_deg);
        JointAnglesRad calculated_joints = cad_solver.inverse_kinematics(target_pose, elbow_guess);
        
        // Use EXPECT_NEAR with a 1e-4 radian tolerance (approx 0.005 degrees) for floating-point IK comparisons
        EXPECT_NEAR(calculated_joints[0], deg2rad(pt.t1_deg), 1e-4) 
            << "IK Theta 1 failed at table index " << i;
            
        EXPECT_NEAR(calculated_joints[1], deg2rad(pt.t2_deg), 1e-4) 
            << "IK Theta 2 failed at table index " << i;
            
        EXPECT_NEAR(calculated_joints[2], deg2rad(pt.t3_deg), 1e-4) 
            << "IK Theta 3 failed at table index " << i;
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}