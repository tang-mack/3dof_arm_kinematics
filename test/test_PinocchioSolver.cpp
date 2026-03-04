#include <gtest/gtest.h>
#include "planar_arm_kinematics/core/PinocchioSolver.h"
#include <memory>
#include <string>
#include <vector>

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
struct SolverTestConfig {
    std::string test_variant_name; 
    bool use_yaml_constructor;
    std::string yaml_filepath;     // Used if use_yaml_constructor is true
    PinocchioSolverConfig struct_config; // Used if use_yaml_constructor is false

    // Add this to make GTest output readable! Avoids very long raw hex of this struct.
    friend std::ostream& operator<<(std::ostream& os, const SolverTestConfig& config) {
        return os << config.test_variant_name;
    }
};

// ==============================================================================
// 2. THE TEST FIXTURE
// This runs automatically before every single TEST_P. It reads the current 
// configuration and builds the solver so the test logic doesn't have to.
// ==============================================================================
class PinocchioParameterizedTest : public ::testing::TestWithParam<SolverTestConfig> {
protected:
    std::unique_ptr<PinocchioSolver> solver_;

    void SetUp() override {
        const auto& params = GetParam();
        if (params.use_yaml_constructor) {
            solver_ = std::make_unique<PinocchioSolver>(params.yaml_filepath);
        } else {
            solver_ = std::make_unique<PinocchioSolver>(params.struct_config);
        }
    }
};

// ==============================================================================
// 3. THE ACTUAL TEST LOGIC
// We use TEST_P instead of TEST. We write the math validation exactly once.
// ==============================================================================
// Test 1: Forward Kinematics at Zero Joint Angles
TEST_P(PinocchioParameterizedTest, ForwardKinematicsZeroAngles) {
    // Set all joint angles to 0.0 radians
    JointAnglesRad joints = {0.0, 0.0, 0.0};
    
    // The solver is already instantiated by SetUp()
    Pose_XY_Yaw pose = solver_->forward_kinematics(joints);
    
    // If the arm is fully extended on the X-axis, X should be 0.7m
    EXPECT_DOUBLE_EQ(pose.x, 0.7);
    EXPECT_DOUBLE_EQ(pose.y, 0.0);
    EXPECT_DOUBLE_EQ(pose.yaw, 0.0);
}

// Test 2: Inverse Kinematics from the Zero Configuration
TEST_P(PinocchioParameterizedTest, InverseKinematicsZeroConfiguration) {
    // Target the fully extended position (assuming congruent 0.3, 0.3, 0.1 lengths)
    Pose_XY_Yaw target{0.7, 0.0, 0.0};
    
    // Pass 30 degrees (in rads) as the elbow guess
    JointAnglesRad q_guess = {0.0, 30.0 * M_PI / 180.0, 0.0};
    JointAnglesRad joints = {0.0, 0.0, 0.0};
    IKStatus status;
    
    bool success = solver_->inverse_kinematics(target, joints, q_guess, status);
    
    EXPECT_TRUE(success);
    EXPECT_EQ(status, IKStatus::SUCCESS);
    
    // // The angles required to reach this should all be 0.0
    // EXPECT_DOUBLE_EQ(joints[0], 0.0);
    // EXPECT_DOUBLE_EQ(joints[1], 0.0);
    // EXPECT_DOUBLE_EQ(joints[2], 0.0);

    std::cout << "Note: Tolerance set to 1e-4 (more forgiving than perfect anaytical IK) to account for Pinocchio's iterative numerical solver precision.\n";
    // Use EXPECT_NEAR with a 1e-4 tolerance instead of EXPECT_DOUBLE_EQ
    EXPECT_NEAR(joints[0], 0.0, 1e-4);
    EXPECT_NEAR(joints[1], 0.0, 1e-4);
    EXPECT_NEAR(joints[2], 0.0, 1e-4);
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
TEST_P(PinocchioParameterizedTest, CadValidationPoints) {
    std::vector<CadDataPoint> cad_points = get_cad_points();

    // Run every row in the table through the solver
    for (size_t i = 0; i < cad_points.size(); ++i) {
        const auto& pt = cad_points[i];
        
        JointAnglesRad joints = {deg2rad(pt.t1_deg), deg2rad(pt.t2_deg), deg2rad(pt.t3_deg)};
        Pose_XY_Yaw pose = solver_->forward_kinematics(joints);
        
        EXPECT_NEAR(pose.x, pt.expected_x_m, 1e-5) 
            << "FK X failed at table index " << i;
        EXPECT_NEAR(pose.y, pt.expected_y_m, 1e-5) 
            << "FK Y failed at table index " << i;
        EXPECT_NEAR(pose.yaw, deg2rad(pt.expected_yaw_deg), 1e-5) 
            << "FK Yaw failed at table index " << i;
    }
}

// Test 4: Inverse Kinematics CAD Ground Truth Validation
TEST_P(PinocchioParameterizedTest, InverseKinematicsCadValidation) {
    std::vector<CadDataPoint> cad_points = get_cad_points();

    // Run every row in the table through the IK solver
    for (size_t i = 0; i < cad_points.size(); ++i) {
        const auto& pt = cad_points[i];
        
        Pose_XY_Yaw target_pose{pt.expected_x_m, pt.expected_y_m, deg2rad(pt.expected_yaw_deg)};
        
        // Pass the expected elbow angle as the guess
        JointAnglesRad q_guess = {0.0, deg2rad(pt.t2_deg), 0.0};
        JointAnglesRad calculated_joints = {0.0, 0.0, 0.0};
        IKStatus status;
        
        bool success = solver_->inverse_kinematics(target_pose, calculated_joints, q_guess, status);

        EXPECT_TRUE(success) << "IK failed to find solution at table index " << i;
        EXPECT_EQ(status, IKStatus::SUCCESS);
        
        EXPECT_NEAR(calculated_joints[0], deg2rad(pt.t1_deg), 1e-4) 
            << "IK Theta 1 failed at table index " << i;
        EXPECT_NEAR(calculated_joints[1], deg2rad(pt.t2_deg), 1e-4) 
            << "IK Theta 2 failed at table index " << i;
        EXPECT_NEAR(calculated_joints[2], deg2rad(pt.t3_deg), 1e-4) 
            << "IK Theta 3 failed at table index " << i;
    }
}

// Test 5: Automated Randomized Forward/Inverse Kinematics Cycle
TEST_P(PinocchioParameterizedTest, RandomizedFkIkCycle) {
    std::mt19937 gen(42);
    
    std::uniform_real_distribution<double> dist_joint_1(deg2rad(-178.0), deg2rad(178.0));
    std::uniform_real_distribution<double> dist_joint_2(deg2rad(-178.0), deg2rad(178.0));
    std::uniform_real_distribution<double> dist_joint_3(deg2rad(-178.0), deg2rad(178.0));

    const int num_iterations = 10000;

    for (int i = 0; i < num_iterations; ++i) {
        JointAnglesRad original_joints = {dist_joint_1(gen), dist_joint_2(gen), dist_joint_3(gen)};

        Pose_XY_Yaw ee_pose = solver_->forward_kinematics(original_joints);

        JointAnglesRad calculated_joints = {0.0, 0.0, 0.0};
        IKStatus status;
        
        // Pass the FULL original joints as the guess to ensure continuous branch selection
        bool success = solver_->inverse_kinematics(ee_pose, calculated_joints, original_joints, status);

        EXPECT_TRUE(success) << "IK failed at iteration " << i;
        EXPECT_EQ(status, IKStatus::SUCCESS);

        EXPECT_NEAR(calculated_joints[0], original_joints[0], 1e-4) 
            << "Joint 0 mismatch at iteration " << i;
        EXPECT_NEAR(calculated_joints[1], original_joints[1], 1e-4) 
            << "Joint 1 mismatch at iteration " << i;
        EXPECT_NEAR(calculated_joints[2], original_joints[2], 1e-4) 
            << "Joint 2 mismatch at iteration " << i;
    }
}

// Test 6: Out of Reach Target Validation
TEST_P(PinocchioParameterizedTest, OutOfReachTarget) {
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

// Add more TEST_P blocks here for Inverse Kinematics, CAD validations, etc.
// They will all automatically run against every configuration (see below for CONFIGURATION GENERATOR).


// ==============================================================================
// 4. CONFIGURATION GENERATOR
// This feeds the variants into the test suite.
// ==============================================================================
std::vector<SolverTestConfig> GenerateTestConfigs() {
    std::string pkg_dir = PKG_SRC_DIR;
    
    // Variant 1: The "Real" Yaml and URDF
    SolverTestConfig real_yaml;
    real_yaml.test_variant_name = "RealYaml";
    real_yaml.use_yaml_constructor = true;
    real_yaml.yaml_filepath = pkg_dir + "/include/planar_arm_kinematics/config/kinematics.yaml";

    // Variant 2: The Mock Congruent Yaml
    SolverTestConfig mock_yaml;
    mock_yaml.test_variant_name = "MockCongruentYaml";
    mock_yaml.use_yaml_constructor = true;
    mock_yaml.yaml_filepath = pkg_dir + "/test/mock_robot_configs/congruent/config/kinematics.yaml";

    // Variant 3: Custom Struct Injection (Congruent URDF + Lookup Speedup True)
    SolverTestConfig mock_struct;
    mock_struct.test_variant_name = "MockStructSpeedup";
    mock_struct.use_yaml_constructor = false;
    mock_struct.struct_config.urdf_filepath = pkg_dir + "/test/mock_robot_configs/congruent/config/urdf/congruent.urdf";
    mock_struct.struct_config.use_lookup_table_speedup = true;

    return {real_yaml, mock_yaml, mock_struct};
}

// Bind the generator to the Fixture
INSTANTIATE_TEST_SUITE_P(
    PinocchioVariants, // Just a prefix name
    PinocchioParameterizedTest,
    ::testing::ValuesIn(GenerateTestConfigs()),
    [](const ::testing::TestParamInfo<SolverTestConfig>& info) {
        return info.param.test_variant_name; // Appends the variant name to the test output
    }
);