#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <random>
#include <functional>
#include <cmath>

// Include the Base Interface
#include "planar_arm_kinematics/core/KinematicSolver.h"

// Include the solver backends
#include "planar_arm_kinematics/core/AnalyticalSolver.h"
#include "planar_arm_kinematics/core/PinocchioSolver.h"

// Fallback just in case the CMake macro fails to inject
#ifndef PKG_SRC_DIR
#define PKG_SRC_DIR "."
#endif

using namespace planar_arm;

// Helper
inline double deg2rad(double deg) {return deg * M_PI / 180.0;}


// ==============================================================================
// 1. FACTORY DEFINITION
// Holds a name for the test output, and a lambda function to build the solver.
// ==============================================================================
struct SolverFactory {
    std::string solver_name;
    std::function<std::unique_ptr<KinematicSolver>()>  create_solver; // function returning std::unique_ptr<KinematicSolver>

    // Tells GTest how to print the name in the terminal
    friend std::ostream& operator<<(std::ostream& os, const SolverFactory& factory) {
        return os << factory.solver_name;
    }
};



// ==============================================================================
// 2. GENERALIZED TEST FIXTURE
// ==============================================================================
class KinematicSolverGeneralizedTest : public ::testing::TestWithParam<SolverFactory> {
protected:
    std::unique_ptr<KinematicSolver> solver_;

    void SetUp() override {
        // Build the solver using the lambda function provided by the factory
        solver_ = GetParam().create_solver();
        ASSERT_NE(solver_, nullptr) << "Factory failed to create the solver!";
    }

};

// ==============================================================================
// 3. THE ACTUAL GENERALIZED TESTS
// ==============================================================================

// Test 1: Zero-Configuration Loop
// Rule: IK(FK(0,0,0)) must cleanly return to (0,0,0)
TEST_P(KinematicSolverGeneralizedTest, ZeroConfigurationCycle) {
    JointAnglesRad zero_angles = {0.0, 0.0, 0.0};

    // Dynamically find where "straight out" is for this specific robot
    Pose_XY_Yaw extended_pose = solver_->forward_kinematics(zero_angles);

    // Provide a slight offset guess so it actually has to do math
    JointAnglesRad q_guess = {0.0, deg2rad(15.0), 0.0};
    JointAnglesRad q_solution = {99.0, 99.0, 99.0};
    IKStatus status;

    bool success = solver_->inverse_kinematics(extended_pose, q_solution, q_guess, status);

    EXPECT_TRUE(success);
    EXPECT_EQ(status, IKStatus::SUCCESS);

    // Dynamic Tolerance Check based on Solver Type
    if (GetParam().solver_name == "PinocchioSolver") {
        std::cout << "[  INFO    ] PinocchioSolver is exempt from strict 1e-4 tolerance at the boundary singularity.\n"
                  << "[  INFO    ] Intentional damping (lambda^2) restricts joint movement to prevent infinite joint velocities.\n";
        
        EXPECT_NEAR(q_solution[0], 0.0, 1e-3);
        EXPECT_NEAR(q_solution[1], 0.0, 1e-3);
        EXPECT_NEAR(q_solution[2], 0.0, 1e-3);
    } 
    else {
        // Analytical and exact solvers must adhere to the strict 1e-4 contract
        EXPECT_NEAR(q_solution[0], 0.0, 1e-4);
        EXPECT_NEAR(q_solution[1], 0.0, 1e-4);
        EXPECT_NEAR(q_solution[2], 0.0, 1e-4);
    }
}

// Test 2: Randomized Continuous FK/IK Cycle
// Rule: FK then IK must return original input angle.
TEST_P(KinematicSolverGeneralizedTest, RandomizedFKCycle) {
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist(deg2rad(-178.0), deg2rad(178.0));
    const int num_iterations = 2500; // Keep it fast for general testing

    for (int i = 0; i < num_iterations; ++i) {
        JointAnglesRad original_joints = {dist(gen), dist(gen), dist(gen)};

        Pose_XY_Yaw ee_pose = solver_->forward_kinematics(original_joints);

        JointAnglesRad calculated_joints = {0.0, 0.0, 0.0};
        IKStatus status;
        
        bool success = solver_->inverse_kinematics(ee_pose, calculated_joints, original_joints, status);

        EXPECT_TRUE(success) << "IK failed at iteration " << i;
        EXPECT_EQ(status, IKStatus::SUCCESS);

        EXPECT_NEAR(calculated_joints[0], original_joints[0], 1e-4);
        EXPECT_NEAR(calculated_joints[1], original_joints[1], 1e-4);
        EXPECT_NEAR(calculated_joints[2], original_joints[2], 1e-4);
    }
    
}

// Test 3: Origin Singularity (X=0, Y=0)
// Rule: Forcing the end-effector into the base motor must NOT crash the solver or return NaNs.
// Note: This probably won't happen because the robot would run into itself, but good to check.
TEST_P(KinematicSolverGeneralizedTest, OriginSingularitySurvival) {
    Pose_XY_Yaw singularity_target{0.0, 0.0, 0.0};
    
    JointAnglesRad q_guess = {0.0, deg2rad(45.0), 0.0};
    JointAnglesRad q_solution = {0.0, 0.0, 0.0};
    IKStatus status;
    
    // We do NOT assert EXPECT_TRUE(success) here. 
    // Depending on the link lengths, the origin might be physically out of reach.
    // The contract just requires that it gracefully handles the math without segfaulting.
    solver_->inverse_kinematics(singularity_target, q_solution, q_guess, status);

    // Verify the math didn't explode into Not-a-Number (NaN) due to division by zero
    EXPECT_FALSE(std::isnan(q_solution[0]));
    EXPECT_FALSE(std::isnan(q_solution[1]));
    EXPECT_FALSE(std::isnan(q_solution[2]));
}



// ==============================================================================
// 4. EASY FACTORY INSTANTIATION
// Add new solvers here to automatically run them through these generalized tests.
// ==============================================================================
std::vector<SolverFactory> GetSolversToTest() {
    std::string pkg_dir = PKG_SRC_DIR;
    std::string mock_yaml = pkg_dir + "/test/mock_robot_configs/congruent/config/kinematics.yaml";

    std::vector<SolverFactory> factories;

    // 1. Analytical Solver
    factories.push_back({
        "AnalyticalSolver",
        [mock_yaml]() { return std::make_unique<AnalyticalSolver>(mock_yaml); }
    });

    // 2. Pinocchio Solver
    factories.push_back({
        "PinocchioSolver",
        [mock_yaml]() { return std::make_unique<PinocchioSolver>(mock_yaml); }
    });

    // 3. NewSolverHere
    // factories.push_back({
    //     "MachineLearningSolver",
    //     [mock_yaml]() { return std::make_unique<MachineLearningSolver>(mock_yaml); }
    // });

    return factories;
}


INSTANTIATE_TEST_SUITE_P(
    AllSolvers,
    KinematicSolverGeneralizedTest,
    ::testing::ValuesIn(GetSolversToTest()),
    [](const ::testing::TestParamInfo<SolverFactory>& info) {
        return info.param.solver_name;
    }
);


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}