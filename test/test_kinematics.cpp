#include <gtest/gtest.h>
#include "planar_arm_kinematics/core/Kinematics.h"
#include "planar_arm_kinematics/core/AnalyticalSolver.h"
#include "planar_arm_kinematics/core/RobotModel.h"
#include <cmath>
#include <vector>
#include <random>

using namespace planar_arm;

// Declared before the tests
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// Helper function to generate a default AnalyticalSolverConfig.
AnalyticalSolverConfig get_default_test_config() {
    AnalyticalSolverConfig config;
    config.urdf_filepath = "dummy_path";
    config.parse_lengths_from = "urdf"; // Forces the solver to use the RobotModel lengths
    config.use_lookup_table_speedup = false;
    config.joint_limits = {{-178.0, 178.0}, {-178.0, 178.0}, {-178.0, 178.0}};
    return config;
}

// Test 1: Forward Kinematics at Zero Configuration
TEST(KinematicsTest, ForwardKinematicsZeroAngles) {
    // Instantiate with default lengths: L1=0.3, L2=0.3, L3=0.1
    RobotModel model(0.3, 0.3, 0.1);
    AnalyticalSolverConfig config = get_default_test_config();
    AnalyticalSolver solver(config, model);
    
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
    RobotModel model(0.3, 0.3, 0.1);
    AnalyticalSolverConfig config = get_default_test_config();
    AnalyticalSolver solver(config, model);
    
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
    RobotModel model(0.3, 0.3, 0.1);
    AnalyticalSolverConfig config = get_default_test_config();
    AnalyticalSolver cad_solver(config, model);

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
    RobotModel model(0.3, 0.3, 0.1);
    AnalyticalSolverConfig config = get_default_test_config();
    AnalyticalSolver solver(config, model);

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
        JointAnglesRad calculated_joints = solver.inverse_kinematics(target_pose, elbow_guess);
        
        // Use EXPECT_NEAR with a 1e-4 radian tolerance (approx 0.005 degrees) for floating-point IK comparisons
        // Rationale: 1e-4 is smaller than many encoder errors
        EXPECT_NEAR(calculated_joints[0], deg2rad(pt.t1_deg), 1e-4) 
            << "IK Theta 1 failed at table index " << i;
            
        EXPECT_NEAR(calculated_joints[1], deg2rad(pt.t2_deg), 1e-4) 
            << "IK Theta 2 failed at table index " << i;
            
        EXPECT_NEAR(calculated_joints[2], deg2rad(pt.t3_deg), 1e-4) 
            << "IK Theta 3 failed at table index " << i;
    }
}

// Test 5: Automated Randomized Forward/Inverse Kinematics Cycle
//
// Take random joint angles falling within joint limits,
// run forward kinematics to get an ee pose, then run inverse kinematics
// to get a joint angle again. Compare new joint angle with original.
TEST(KinematicsTest, RandomizedFkIkCycle) {
    RobotModel model(0.3, 0.3, 0.1);
    AnalyticalSolverConfig config = get_default_test_config();
    AnalyticalSolver solver(config, model);

    // Set a fixed seed for reproducible test results across different environments
    std::mt19937 gen(42);
    
    // Distribution range: -178 to +178 degrees (in radians)
    // Rationale: if the FK/IK works on a very large joint limit range, it should work for smaller ones too.
    std::uniform_real_distribution<double> dist_joint_1(deg2rad(-178.0), deg2rad(178.0));
    std::uniform_real_distribution<double> dist_joint_2(deg2rad(-178.0), deg2rad(178.0));
    std::uniform_real_distribution<double> dist_joint_3(deg2rad(-178.0), deg2rad(178.0));

    const int num_iterations = 10000;

    for (int i = 0; i < num_iterations; ++i) {
        // 1. Generate random joint angles
        JointAnglesRad original_joints = {dist_joint_1(gen), dist_joint_2(gen), dist_joint_3(gen)};

        // 2. Compute Forward Kinematics to get a valid, reachable target pose
        Pose_XY_Yaw ee_pose = solver.forward_kinematics(original_joints);

        // 3. Compute Inverse Kinematics
        // The original elbow joint is passed as the guess to force the solver
        // (if we had elbow-up vs. down, we want to get the same out).
        JointAnglesRad calculated_joints = solver.inverse_kinematics(ee_pose, original_joints[1]);

        // 4. Validate the output matches the input
        EXPECT_NEAR(calculated_joints[0], original_joints[0], 1e-4) 
            << "Joint 0 mismatch at iteration " << i;
        EXPECT_NEAR(calculated_joints[1], original_joints[1], 1e-4) 
            << "Joint 1 mismatch at iteration " << i;
        EXPECT_NEAR(calculated_joints[2], original_joints[2], 1e-4) 
            << "Joint 2 mismatch at iteration " << i;
    }
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}