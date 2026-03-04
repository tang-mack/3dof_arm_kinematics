#include "planar_arm_kinematics/core/PinocchioSolver.h"
#include "planar_arm_kinematics/core/YamlReader.h"

#include <pinocchio/parsers/urdf.hpp> // Pinocchio URDF parser

#include <pinocchio/algorithm/kinematics.hpp> // forward kinematics
#include <pinocchio/algorithm/frames.hpp> // forward kinematics

#include <pinocchio/algorithm/jacobian.hpp> // Add for inverse kinematics
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <cmath> // For std::atan2

#include <stdexcept>
#include <filesystem>
#include <iostream>

namespace planar_arm {

// Constructor 1 (typical): Fill config struct using Yaml
PinocchioSolver::PinocchioSolver(const std::string& yaml_filepath) {
    // Pinocchio initialization

    YamlReader yaml_reader;
    yaml_reader.load(yaml_filepath);

    config_.urdf_filepath = yaml_reader.get_global_param<std::string>("urdf_filepath");
    config_.use_lookup_table_speedup = yaml_reader.get_class_specific_param<bool>("PinocchioSolver", "use_lookup_table_speedup");

    // Resolve the URDF path relative to the Yaml file to an absolute path
    std::filesystem::path yaml_dir = std::filesystem::path(yaml_filepath).parent_path();
    std::filesystem::path absolute_urdf_path = yaml_dir / config_.urdf_filepath;

    load_urdf(absolute_urdf_path.string());


}

// Constructor 2: Inject config struct directly
PinocchioSolver::PinocchioSolver(const PinocchioSolverConfig& config) : config_(config) {
    // For unit tests, we assume the struct provides a valid path
    load_urdf(config_.urdf_filepath);
}

void PinocchioSolver::load_urdf(const std::string& absolute_urdf_path) {
    try {
        // Build the kinematic tree
        pinocchio::urdf::buildModel(absolute_urdf_path, model_);
    } catch (const std::invalid_argument& e) {
        throw std::runtime_error("[PinocchioSolver] Failed to parse URDF: " + absolute_urdf_path + " | " + e.what());
    }

    // Initialize the working memory cache for the math algorithms
    data_ = pinocchio::Data(model_);

    // Look up the integer ID for the end-effector frame so we don't have to search for it during real-time loops
    const std::string ee_name = "end_effector";
    if (model_.existFrame(ee_name)) {
        ee_frame_id_ = model_.getFrameId(ee_name);
        std::cout << "[PinocchioSolver] Successfully loaded URDF. End-effector frame ID: " << ee_frame_id_ << std::endl;
    } else {
        throw std::runtime_error("[PinocchioSolver] Frame '" + ee_name + "' not found in URDF.");
    }
}

Pose_XY_Yaw PinocchioSolver::forward_kinematics(const JointAnglesRad& joints) const {
    // Load joint angles into Eigen
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);

    // Safely copy our 3 joints into Eigen
    for (int i = 0; i < std::min(3, (int)model_.nq); ++i) {
        q[i] = joints[i];
    }

    // Run forward kinematics (write to data_ cache)
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacement(model_, data_, ee_frame_id_);

    // Extract the 2D Pose from the end-effector's SE(3) transform (oMf = Origin to Frame)
    const pinocchio::SE3& ee_transform = data_.oMf[ee_frame_id_];

    Pose_XY_Yaw ee_pose;
    ee_pose.x = ee_transform.translation().x();
    ee_pose.y = ee_transform.translation().y();

    // Extract Yaw by taking the arctangent of the 2D rotation components
    // R(1,0) corresponds to sin(yaw), R(0,0) corresponds to cos(yaw)
    const auto& R = ee_transform.rotation();
    ee_pose.yaw = std::atan2(R(1,0), R(0,0));

    return ee_pose;    
}

/// @brief Levenberg-Marquadt inverse kinematics implementation
/// @details Why LM: LM (aka damped least squares) is good for dealing with singularities. It's a technique used by humanoid companies.
/// It's advantageous when moving around and continually solving IK at a high rate: you simply feed in the previous
/// solution as an initial guess. Not great if you don't have an initial guess.
///
/// Literature summary:
///     https://www.mathworks.com/help/robotics/ug/inverse-kinematics-algorithms.html#bve7apt
///     Summary: performs well when a good initial guess can be given (ie. finding IK poses along a known desired trajectory)
///
/// @param ee_pose 
/// @param guess_elbow_joint 
/// @return the joint angles in [rad]
bool PinocchioSolver::inverse_kinematics(const Pose_XY_Yaw& ee_target, JointAnglesRad& q_solution, const JointAnglesRad& q_guess, IKStatus& status) const {

    JointAnglesRad original_q_solution = q_solution; // Store a copy of the original input: if IK fails, we return what was passed in

    // ========================================================================
    // ---- FAILURE CHECK: OUT OF REACH (KINEMATIC TREE PRE-CHECK) ----
    // Dynamically calculate the absolute maximum theoretical reach of this URDF
    // by walking up the parent tree from the end-effector to the base.
    double max_reach = model_.frames[ee_frame_id_].placement.translation().norm(); // [m]
    pinocchio::JointIndex current_joint = model_.frames[ee_frame_id_].parentJoint;
    
    while (current_joint > 0) { // 0 is the universe/world base
        max_reach += model_.jointPlacements[current_joint].translation().norm();
        current_joint = model_.parents[current_joint];
    }

    // Calculate the straight-line distance (L2 norm) to the target from the base
    double target_distance = std::sqrt(ee_target.x * ee_target.x + ee_target.y * ee_target.y); // [m]
    
    // If the target is further than the unrolled arm lengths (plus tiny epsilon), exit early
    if (target_distance > max_reach + 1e-4) {
        q_solution = original_q_solution;
        status = IKStatus::OUT_OF_REACH;
        return false;
    }
    // ========================================================================

    // Build the world_T_target SE(3) transformation matrix
    Eigen::Matrix3d world_R_target = Eigen::AngleAxisd(ee_target.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(); // world_R_ee = Rotate by "yaw" around Z_world
    Eigen::Vector3d world_p_target(ee_target.x, ee_target.y, 0.0);
    pinocchio::SE3 world_T_target(world_R_target, world_p_target);

    // Initialize q with our shoulder guess, elbow guess, wrist guess
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
    for (int i = 0; i < std::min(3, (int)model_.nq); ++i) {
        q[i] = q_guess[i];
    }

    // Iterative Solver Settings
    const double error_tol = 1e-4; // Error tolerance [radians]
    const int ITERATIONS_MAX = 500; // Maximum allowed iterations
    const double damping = 1e-6; // Damping factor for pseudo-inverse (prevents singularity crashes). Approaches mention lambda, here, damping = lambda*lambda.

    pinocchio::Data::Matrix6x J_local(6, model_.nv); // Local to ee frame: x_dot_wrt_ee_frame = J_local(q)*q_dot
    Eigen::VectorXd q_dot_step(model_.nv); // small joint velocity "step" to take

    // The Gauss-Newton Iteration Loop
    for (int i = 0; i  < ITERATIONS_MAX; ++i) {
        // Calculate where the arm currently is
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacement(model_, data_, ee_frame_id_);

        // ---- Calculate the spatial error between current ee and target ee ----
        const pinocchio::SE3 world_T_current = data_.oMf[ee_frame_id_]; // oMf: Origin to Frame
        const pinocchio::SE3 current_T_target = world_T_current.actInv(world_T_target); // current_T_target = (world_T_current)^-1 * world_T_target
        // You can't just subtract two rotation matrices or T matrices: they exist on a manifold (ie. two points on the earth cannot be connected
        // with a straight line, that would go "underground")
        // But you can use the matrix logarithm to convert to 6 numbers: log(current_T_target) = [XYZ velocity, angular velocity]. 
        // This [XYZ velocity, angular velocity] quantifies the difference as a velocity in one unit of time.
        // If the velocity is large, current_T_target is a massive rotation matrix + massive translation, and small velocity means small T.
        Eigen::VectorXd linear_vel_angular_vel_error = pinocchio::log6(current_T_target).toVector();

        // But we need to Slice this 6D velocity error [vx, vy, vz, wx, wy, wz] down to strictly planar dimensions [vx, vy, wz], indexes 0, 1, and 5.
        Eigen::Vector3d linear_vel_angular_vel_error_planar_only;
        linear_vel_angular_vel_error_planar_only << linear_vel_angular_vel_error(0), 
                                                    linear_vel_angular_vel_error(1), 
                                                    linear_vel_angular_vel_error(5);

        // ---- SUCCESS CHECK ----
        // Check if we have reached the target ee
        // if (linear_vel_angular_vel_error.norm() < error_tol) {
        if (linear_vel_angular_vel_error_planar_only.norm() < error_tol) {
            
            // Package and wrap-to-pi the results
            for (int j = 0; j < std::min(3, (int)model_.nq); ++j) {
                // Wrap angles strictly between [-pi, pi]
                double angle = std::fmod(q[j], 2.0 * M_PI);
                if (angle > M_PI) angle -= 2.0 * M_PI;
                if (angle < -M_PI) angle += 2.0 * M_PI;
                q_solution[j] = angle;
            }
            
            status = IKStatus::SUCCESS;
            return true;
        }

        // Compute the Jacobian in the LOCAL frame of the end-effector. x_dot_wrt_ee_frame = J_local(q)*q_dot
        J_local.setZero();
        pinocchio::computeFrameJacobian(model_, data_, q, ee_frame_id_, pinocchio::LOCAL, J_local);

        // Slice the Jacobian down to the same 3 planar rows [vx, vy, wz]
        Eigen::MatrixXd J_planar(3, model_.nv);
        J_planar.row(0) = J_local.row(0); // vx
        J_planar.row(1) = J_local.row(1); // vy
        J_planar.row(2) = J_local.row(5); // wz

        // // Compute the damped pseudo-inverse step
        // // q_dot_step = J.T * (J * J.T + damping * I)^-1 * error
        // pinocchio::Data::Matrix6 JJt; // J*J.T
        // // JJt.noalias() = J_local * J_local.transpose();
        // Eigen::MatrixXd JtJ = J_local.transpose() * J_local;
        // JJt.diagonal().array() += damping;

        // Compute the damped left pseudo-inverse step using the planar matrices
        Eigen::MatrixXd JtJ = J_planar.transpose() * J_planar;
        JtJ.diagonal().array() += damping;

        // q_dot_step.noalias() = J_local.transpose() * JJt.ldlt().solve(linear_vel_angular_vel_error);
        q_dot_step.noalias() = JtJ.ldlt().solve(J_planar.transpose() * linear_vel_angular_vel_error_planar_only);

        // Apply the velocity step to our joint angles
        q = pinocchio::integrate(model_, q, q_dot_step);
    }

    // ---- FAILURE CHECK ----
    // If the loop finishes without breaking, it reached max iterations without converging
    q_solution = original_q_solution; // Failure behavior: Return what was passed in
    status = IKStatus::MAX_ITERATIONS_REACHED;
    return false;
}

} // namespace planar_arm