#include <gtest/gtest.h>
#include "planar_arm_kinematics/core/Kinematics.h"
#include "planar_arm_kinematics/core/AnalyticalSolver.h"
#include "planar_arm_kinematics/core/RobotModel.h"
#include <cmath>
#include <vector>
#include <random>
#include <memory>
#include <string>

using namespace planar_arm;

// Declared before the tests
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// ==============================================================================
// 1. PARAMETER DEFINITION
// Define a struct that holds all the ways we want to initialize the solver.
// ==============================================================================
struct AnalyticalTestConfig {
    std::string test_variant_name;
    AnalyticalSolverConfig config;
    std::vector<double> link_lengths;

    // Add this to make GTest output readable! Avoids very long raw hex of this struct.
    friend std::ostream& operator<<(std::ostream& os, const AnalyticalTestConfig& config) {
        return os << config.test_variant_name;
    }
};

// ==============================================================================
// 2. THE TEST FIXTURE
// This runs automatically before every single TEST_P.
// ==============================================================================
class AnalyticalParameterizedTest : public ::testing::TestWithParam<AnalyticalTestConfig> {
protected:
    std::unique_ptr<RobotModel> model_;
    std::unique_ptr<AnalyticalSolver> solver_;

    void SetUp() override {
        const auto& params = GetParam();
        model_ = std::make_unique<RobotModel>(params.link_lengths);
        solver_ = std::make_unique<AnalyticalSolver>(params.config, *model_);
    }
};

// ==============================================================================
// 3. THE ACTUAL TEST LOGIC
// ==============================================================================

// Test 1: Forward Kinematics at Zero Configuration
TEST_P(AnalyticalParameterizedTest, ForwardKinematicsZeroAngles) {
    // Set all joint angles to 0.0 radians
    JointAnglesRad joints = {0.0, 0.0, 0.0};
    
    Pose_XY_Yaw pose = solver_->forward_kinematics(joints);
    
    // If the arm is fully extended on the X-axis, X should be the sum of all link lengths (0.7m)
    EXPECT_DOUBLE_EQ(pose.x, 0.7);
    EXPECT_DOUBLE_EQ(pose.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.yaw, 0.0);
}

// Test 2: Inverse Kinematics from the Zero Configuration
TEST_P(AnalyticalParameterizedTest, InverseKinematicsZeroConfiguration) {
    // Target the fully extended position
    Pose_XY_Yaw target{0.7, 0.0, 0.0};

    // Call the solver
    JointAnglesRad q_guess = {0.0, deg2rad(30.0), 0.0}; 
    JointAnglesRad q_solution = {0.0, 0.0, 0.0};
    IKStatus status;
    bool success = solver_->inverse_kinematics(target, q_solution, q_guess, status);

    // Verify the solver successfully found a solution
    EXPECT_TRUE(success);
    EXPECT_EQ(status, IKStatus::SUCCESS);
    
    // The angles required to reach this should all be 0.0
    EXPECT_DOUBLE_EQ(q_solution[0], 0.0);
    EXPECT_DOUBLE_EQ(q_solution[1], 0.0);
    EXPECT_DOUBLE_EQ(q_solution[2], 0.0);
}

// Struct to hold CAD ground truth data points
struct CadDataPoint {
    double t1_deg;
    double t2_deg;
    double t3_deg;
    double expected_x_m;
    double expected_y_m;
    double expected_yaw_deg;
};

// Helper function to provide the CAD table
std::vector<CadDataPoint> get_cad_points() {
    return {
        // t1 (deg), t2 (deg), t3 (deg), X (m),         Y (m),        Yaw (deg)
        { 17.0,      60.0,     -70.0,    453.631e-3,    392.209e-3,   7.0},
        { -17.0,     60.0,     -70.0,    595.39819e-3,  71.48895e-3, -27.0}
        // Add more rows here...
    };
}

// Test 3: CAD Ground Truth Validation, Single Points (Table Format)
TEST_P(AnalyticalParameterizedTest, CadValidationPoints) {
    std::vector<CadDataPoint> cad_points = get_cad_points();

    // Run every row in the table through the solver
    for (size_t i = 0; i < cad_points.size(); ++i) {
        const auto& pt = cad_points[i];
        
        // Convert degrees to radians right before feeding into the solver
        JointAnglesRad joints = {deg2rad(pt.t1_deg), deg2rad(pt.t2_deg), deg2rad(pt.t3_deg)};
        Pose_XY_Yaw pose = solver_->forward_kinematics(joints);
        
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
TEST_P(AnalyticalParameterizedTest, InverseKinematicsCadValidation) {
    std::vector<CadDataPoint> cad_points = get_cad_points();

    // Run every row in the table through the IK solver
    for (size_t i = 0; i < cad_points.size(); ++i) {
        const auto& pt = cad_points[i];
        
        Pose_XY_Yaw target_pose{pt.expected_x_m, pt.expected_y_m, deg2rad(pt.expected_yaw_deg)};
        
        // Pass the expected elbow angle as the guess to ensure the solver selects the matching configuration
        JointAnglesRad q_guess = {0.0, deg2rad(pt.t2_deg), 0.0};
        JointAnglesRad calculated_joints = {0.0, 0.0, 0.0};
        IKStatus status;
        
        bool success = solver_->inverse_kinematics(target_pose, calculated_joints, q_guess, status);
        
        // Verify the solver successfully found a solution before checking the math
        EXPECT_TRUE(success) << "IK failed to find solution at table index " << i;
        EXPECT_EQ(status, IKStatus::SUCCESS);
        
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
TEST_P(AnalyticalParameterizedTest, RandomizedFkIkCycle) {
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
        Pose_XY_Yaw ee_pose = solver_->forward_kinematics(original_joints);

        // 3. Compute Inverse Kinematics
        JointAnglesRad calculated_joints = {0.0, 0.0, 0.0};
        IKStatus status;
        
        bool success = solver_->inverse_kinematics(ee_pose, calculated_joints, original_joints, status);

        // Ensure the solver didn't throw a failure flag
        EXPECT_TRUE(success) << "IK failed at iteration " << i;
        EXPECT_EQ(status, IKStatus::SUCCESS);

        // 4. Validate the output matches the input
        EXPECT_NEAR(calculated_joints[0], original_joints[0], 1e-4) 
            << "Joint 0 mismatch at iteration " << i;
        EXPECT_NEAR(calculated_joints[1], original_joints[1], 1e-4) 
            << "Joint 1 mismatch at iteration " << i;
        EXPECT_NEAR(calculated_joints[2], original_joints[2], 1e-4) 
            << "Joint 2 mismatch at iteration " << i;
    }
}

// ==============================================================================
// 4. CONFIGURATION GENERATOR
// ==============================================================================
std::vector<AnalyticalTestConfig> GenerateAnalyticalTestConfigs() {
    AnalyticalTestConfig base_config;
    base_config.config.urdf_filepath = "dummy_path";
    base_config.config.link_length_source = "";
    base_config.config.joint_limits = {{-178.0, 178.0}, {-178.0, 178.0}, {-178.0, 178.0}};
    base_config.link_lengths = {0.3, 0.3, 0.1}; // Default CAD lengths

    // Variant 1: Standard
    AnalyticalTestConfig standard = base_config;
    standard.test_variant_name = "StandardConfig";
    standard.config.use_lookup_table_speedup = false;

    // Variant 2: Lookup Table Speedup
    AnalyticalTestConfig lookup_speedup = base_config;
    lookup_speedup.test_variant_name = "LookupSpeedupConfig";
    lookup_speedup.config.use_lookup_table_speedup = true;

    return {standard, lookup_speedup};
}

// Bind the generator to the Fixture
INSTANTIATE_TEST_SUITE_P(
    AnalyticalVariants,
    AnalyticalParameterizedTest,
    ::testing::ValuesIn(GenerateAnalyticalTestConfigs()),
    [](const ::testing::TestParamInfo<AnalyticalTestConfig>& info) {
        return info.param.test_variant_name; 
    }
);

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}