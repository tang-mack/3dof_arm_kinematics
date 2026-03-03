#include "planar_arm_kinematics/core/PinocchioSolver.h"
#include "planar_arm_kinematics/core/YamlReader.h"

#include <pinocchio/parsers/urdf.hpp> // Pinocchio URDF parser

#include <pinocchio/algorithm/kinematics.hpp> // forward kinematics
#include <pinocchio/algorithm/frames.hpp> // forward kinematics
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
    // return Pose_XY_Yaw{0.0, 0.0, 0.0};
}

JointAnglesRad PinocchioSolver::inverse_kinematics(const Pose_XY_Yaw& target, const double guess_elbow_joint) const {
    return JointAnglesRad{0.0, 0.0, 0.0};
}

} // namespace planar_arm