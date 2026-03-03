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

// Add more TEST_P blocks here for Inverse Kinematics, CAD validations, etc.
// They will all automatically run against every configuration!


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