#include <gtest/gtest.h>
#include "planar_arm_kinematics/core/Kinematics.h"
#include "planar_arm_kinematics/core/AnalyticalSolver.h"
#include "planar_arm_kinematics/core/RobotModel.h"
#include <cmath>
#include <vector>
#include <random>
#include <memory>
#include <string>

// Fallback just in case the CMake macro fails to inject
#ifndef PKG_SRC_DIR
#define PKG_SRC_DIR "."
#endif

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
    bool use_yaml_constructor;
    std::string yaml_filepath;
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

        // Behavior to test both AnalyticalSolver constructors depending on variant
        if (params.use_yaml_constructor == true) {
            // Call constructor with Yaml
            solver_ = std::make_unique<AnalyticalSolver>(params.yaml_filepath);
        }
        else {
            // Call constructor with config struct inject and RobotModel inject
            model_ = std::make_unique<RobotModel>(params.link_lengths);
            solver_ = std::make_unique<AnalyticalSolver>(params.config, *model_);
        }
    }
};

// ==============================================================================
// 3. THE ACTUAL TEST LOGIC
// ==============================================================================

// Test 1: Forward Kinematics at Zero Configuration
TEST_P(AnalyticalParameterizedTest, ForwardKinematicsZeroAngles) {
    if (GetParam().test_variant_name == "RealYaml") {
        GTEST_SKIP() << "Skipping ForwardKinematicsZeroAngles for RealYaml:  we rely on putting robots into mock_robot_configs folder for this test.";
    }

    // Set all joint angles to 0.0 radians
    JointAnglesRad joints = {0.0, 0.0, 0.0};
    
    Pose_XY_Yaw pose = solver_->forward_kinematics(joints);
    
    // If the arm is fully extended on the X-axis, X should be the sum of all link lengths (0.7m)
    EXPECT_DOUBLE_EQ(pose.x, 0.7);
    EXPECT_DOUBLE_EQ(pose.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.yaw, 0.0);
}

// Test 2: Inverse Kinematics from the Zero Configuration - Dynamic for changing Yaml link lengths
// Note: This test still runs if Yaml link lengths are changed. Checks IK( FK(all zero joints) ) == all zero joints.
TEST_P(AnalyticalParameterizedTest, InverseKinematicsZeroConfiguration) {

    // 1. DYNAMIC TARGET GENERATION: 
    // Ask the solver where the arm is when fully extended, regardless of link lengths
    JointAnglesRad zero_angles = {0.0, 0.0, 0.0};
    Pose_XY_Yaw dynamic_target = solver_->forward_kinematics(zero_angles);

    // 2. Call the IK solver
    JointAnglesRad q_guess = {0.0, deg2rad(30.0), 0.0}; 
    JointAnglesRad q_solution = {99.0, 99.0, 99.0}; // Sentinel values
    IKStatus status;
    bool success = solver_->inverse_kinematics(dynamic_target, q_solution, q_guess, status);

    // Verify the solver successfully found a solution
    EXPECT_TRUE(success);
    EXPECT_EQ(status, IKStatus::SUCCESS);
    
    // The angles required to reach this should all be 0.0
    // Rationale for using EXPECT_NEAR instead of EXPECT_DOUBLE_EQ: chaining together std::cos, std::sin, std::acos,
    // can cause numerical error. 1e-5 radians is decent.
    EXPECT_NEAR(q_solution[0], 0.0, 1e-5);
    EXPECT_NEAR(q_solution[1], 0.0, 1e-5);
    EXPECT_NEAR(q_solution[2], 0.0, 1e-5);
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
    if (GetParam().test_variant_name == "RealYaml") {
        GTEST_SKIP() << "Skipping fixed CAD Ground Truth tests for FK the RealYaml:  we rely on putting robots into mock_robot_configs folder for this test.";
    }

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
    if (GetParam().test_variant_name == "RealYaml") {
        GTEST_SKIP() << "Skipping fixed CAD Ground Truth tests for IK for the RealYaml:  we rely on putting robots into mock_robot_configs folder for this test.";
    }

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

// Test 6: Out of Reach Target Validation
TEST_P(AnalyticalParameterizedTest, OutOfReachTarget) {
    // Target a coordinate 5 meters away (robot max reach is 0.7m)
    Pose_XY_Yaw impossible_target{5.0, 5.0, 0.0};
    
    // Provide a valid guess, and fill the solution array with sentinel garbage values
    JointAnglesRad q_guess = {0.0, deg2rad(30.0), 0.0};
    JointAnglesRad q_solution = {99.0, 99.0, 99.0};
    JointAnglesRad q_solution_original = q_solution; // Store copy
    IKStatus status;
    
    bool success = solver_->inverse_kinematics(impossible_target, q_solution, q_guess, status);
    
    // Verify the solver gracefully failed
    EXPECT_FALSE(success);
    
    // Verify the status enum correctly identified the reason
    EXPECT_EQ(status, IKStatus::OUT_OF_REACH);
    
    // Verify the output array safely reverted being completely untouched,
    // rather than handing back NaNs or mathematical garbage.
    EXPECT_DOUBLE_EQ(q_solution[0], q_solution_original[0]);
    EXPECT_DOUBLE_EQ(q_solution[1], q_solution_original[1]);
    EXPECT_DOUBLE_EQ(q_solution[2], q_solution_original[2]);
}

// Test 7: Joint Limit Violation Validation
TEST_P(AnalyticalParameterizedTest, JointLimitViolation) {
    if (GetParam().test_variant_name == "RealYaml") {
        GTEST_SKIP() << "Skipping JointLimitViolation for RealYaml: joint breaking CAD points are *specific* to a certain robot, we rely on putting robots into mock_robot_configs folder for this test.";
    }

    // Struct to hold CAD data points that are physically reachable (within 0.7m or total robot arm length)
    // but mathematically require violating the (ie. [-178, 178]) degree joint limits.
    //
    // NOTE: These CAD points assume [-178 178], and hence will go beyond (ie. -178.5).
    // However this test still works if you change the joint limit dynamically in say the yaml (ie. [-150 150]),
    // but within reason, we assume you won't go beyond 178 degrees because that's already fairly aggressive.
    // Unless you have slip rings, 178 is decent.
    //
    struct LimitDataPoint {
        double target_x_m;
        double target_y_m;
        double target_yaw_deg;
        double guess_t2_deg; // Elbow guess to force the specific out-of-bounds configuration
    };

    std::vector<LimitDataPoint> limit_points = {
        // Target X, Target Y, Target Yaw, Elbow Guess (deg)
        // Example: An ee target that forces the wrist (joint 3) to bend to -179 degrees
        { -544.16603e-3, -251.97819e-3, -189.0, 60.0 } 
        // Add more CAD ground truth limit-breaking points here...
    };

    for (size_t i = 0; i < limit_points.size(); ++i) {
        const auto& pt = limit_points[i];
        
        Pose_XY_Yaw target_pose{pt.target_x_m, pt.target_y_m, deg2rad(pt.target_yaw_deg)};
        
        // Pass the elbow guess to guide the solver
        JointAnglesRad q_guess = {0.0, deg2rad(pt.guess_t2_deg), 0.0};
        JointAnglesRad q_solution = {99.0, 99.0, 99.0}; // Sentinel garbage values
        IKStatus status;
        
        bool success = solver_->inverse_kinematics(target_pose, q_solution, q_guess, status);
        
        // Verify the solver gracefully failed
        EXPECT_FALSE(success) << "IK incorrectly succeeded at table index " << i;
        
        // Verify the status enum correctly identified the joint limit violation
        EXPECT_EQ(status, IKStatus::JOINT_LIMIT_VIOLATION) 
            << "Incorrect failure status at table index " << i;
        
        // Verify the output array safely reverted to the guess
        EXPECT_DOUBLE_EQ(q_solution[0], q_solution[0]);
        EXPECT_DOUBLE_EQ(q_solution[1], q_solution[1]);
        EXPECT_DOUBLE_EQ(q_solution[2], q_solution[2]);
    }
}

// Add more TEST_P blocks here for Inverse Kinematics, CAD validations, etc.
// They will all automatically run against every configuration (see below for CONFIGURATION GENERATOR).

// ==============================================================================
// 4. CONFIGURATION GENERATOR
// ==============================================================================
std::vector<AnalyticalTestConfig> GenerateAnalyticalTestConfigs() {
    std::string pkg_dir = PKG_SRC_DIR; // Ensure this macro is defined at the top of your file!
    std::vector<AnalyticalTestConfig> configs;

    // Variant 1: Mock YAML (The Anchor for CAD tests)
    AnalyticalTestConfig mock_yaml;
    mock_yaml.test_variant_name = "MockCongruentYaml";
    mock_yaml.use_yaml_constructor = true;
    mock_yaml.yaml_filepath = pkg_dir + "/test/mock_robot_configs/congruent/config/kinematics.yaml";
    configs.push_back(mock_yaml);

    // Variant 2: Real YAML (The Physical Robot)
    AnalyticalTestConfig real_yaml;
    real_yaml.test_variant_name = "RealYaml";
    real_yaml.use_yaml_constructor = true;
    real_yaml.yaml_filepath = pkg_dir + "/include/planar_arm_kinematics/config/kinematics.yaml";
    configs.push_back(real_yaml);

    // Variant 3: Standard Struct Injection (Testing the raw math with CAD defaults)
    AnalyticalTestConfig standard;
    standard.test_variant_name = "MockStructStandard";
    standard.use_yaml_constructor = false;
    standard.config.urdf_filepath = "dummy_path";
    standard.config.link_length_source = "";
    standard.config.use_lookup_table_speedup = false;
    standard.config.joint_limits = {{deg2rad(-178.0), deg2rad(178.0)}, {deg2rad(-178.0), deg2rad(178.0)}, {deg2rad(-178.0), deg2rad(178.0)}};
    standard.link_lengths = {0.3, 0.3, 0.1}; 
    configs.push_back(standard);

    return configs;
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