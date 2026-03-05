#include <cmath>
#include "planar_arm_kinematics/core/AnalyticalSolver.h"

#include <algorithm> // for std::clamp
#include <iostream>
#include <planar_arm_kinematics/core/MiniURDFParser.h>
#include <filesystem> // for getting URDF filepath

namespace planar_arm {

// Constructor 1: Yaml-based constructor for typical usage
AnalyticalSolver::AnalyticalSolver(const std::string& yaml_filepath) {
        
    yaml_reader_.load(yaml_filepath); // load() will throw if yaml does not exist

    // Parse Class-specific Yaml Parameters (ie. nested under AnalyticalSolver within the yaml file)
    config_.link_length_source = yaml_reader_.get_class_specific_param<std::string>("AnalyticalSolver", "link_length_source"); // "urdf" or "yaml"
    std::cout << "link_length_source: " << config_.link_length_source << std::endl;
    config_.link_lengths = yaml_reader_.get_class_specific_param<std::vector<double>>("AnalyticalSolver", "link_lengths");
    config_.use_lookup_table_speedup = yaml_reader_.get_class_specific_param<bool>("AnalyticalSolver", "use_lookup_table_speedup");

    // Get URDF filepath if we're using it, std::nullopt otherwise
    if (config_.link_length_source == "urdf") {
        config_.urdf_filepath = yaml_reader_.get_global_param<std::string>("urdf_filepath");
    }
    else if (config_.link_length_source == "yaml") {
        config_.urdf_filepath = std::nullopt;
    }
    else {
        throw std::runtime_error("[AnalyticalSolver] YAML link_length_source must be urdf or yaml.");
    }

    // Parse Joint Limits from Yaml
    auto limits_deg = yaml_reader_.get_class_specific_param<std::vector<std::vector<double>>>("AnalyticalSolver", "joint_limits");
    config_.joint_limits.clear();
    config_.joint_limits.resize(limits_deg.size());
    
    // Convert each joint's limits from degrees to radians
    for (size_t i = 0; i < limits_deg.size(); ++i) {
        config_.joint_limits.at(i) = { limits_deg.at(i).at(0) * M_PI / 180.0, limits_deg.at(i).at(1) * M_PI / 180.0 };
    }

    std::cout << "[AnalyticalSolver] Yaml loaded successfully, filepath used was: " << yaml_filepath << std::endl;

    
    if (config_.link_length_source == "yaml") { // YAML overrides URDF link lengths logic
        if (config_.link_lengths.size() == 3) {

            model_ = RobotModel(config_.link_lengths);

        } else {
            throw std::runtime_error("[AnalyticalSolver] YAML link_lengths must contain exactly 3 values.");
        }
    }
    else if (config_.link_length_source == "urdf") { // URDF provides link lengths

        // --- URDF PATH RESOLUTION ---
        std::filesystem::path yaml_dir = std::filesystem::path(yaml_filepath).parent_path(); // Get the directory containing the YAML file
        std::filesystem::path absolute_urdf_path = yaml_dir / config_.urdf_filepath.value(); // Append the relative URDF path from the YAML config
        std::cout << "[AnalyticalSolver] Resolved absolute URDF path: " << absolute_urdf_path << std::endl;
        auto lengths = MiniURDFParser::parse_link_lengths(absolute_urdf_path.string()); // Pass the absolute string to the parser
        // ---------------------------------

        if (lengths.size() != 3) {
            throw std::runtime_error("[AnayticalSolver] URDF must provide exactly 3 link lengths, " + std::to_string(lengths.size()) + " were provided.");
        }

        model_ = RobotModel(lengths); // Set lengths based on URDF

        model_.print_link_lengths();
    }
    
}

// Constructor 2: Struct-based constructor mainly for Unit Tests
AnalyticalSolver::AnalyticalSolver(const AnalyticalSolverConfig& config, const RobotModel& model)
    : config_(config), model_(model)
{
    // Do nothing
}

Pose_XY_Yaw AnalyticalSolver::forward_kinematics(const JointAnglesRad& joint_angles) const {

    double L1 = model_.get_length(0);
    double L2 = model_.get_length(1);
    double L3 = model_.get_length(2);

    double q1 = joint_angles.at(0);
    double q2 = joint_angles.at(1);
    double q3 = joint_angles.at(2);

    double x = L1 * std::cos(q1) + L2 * std::cos(q1 + q2) + L3 * std::cos(q1 + q2 + q3);
    double y = L1 * std::sin(q1) + L2 * std::sin(q1 + q2) + L3 * std::sin(q1 + q2 + q3);
    double theta = q1 + q2 + q3;

    Pose_XY_Yaw pose{x, y, theta};

    return pose;
}

double AnalyticalSolver::wrap_to_pi(const double angle) const {
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool AnalyticalSolver::inverse_kinematics(const Pose_XY_Yaw& ee_target, JointAnglesRad& q_solution, const JointAnglesRad& q_guess, IKStatus& status) const {
    // std::cout << "[AnalyticalSolver] inverse_kinematics called" << std::endl;

    JointAnglesRad original_q_solution = q_solution; // Store what was passed in: if IK fails, we pass the answer back completely untouched.

    double L1 = model_.get_length(0);
    double L2 = model_.get_length(1);
    double L3 = model_.get_length(2);

    double x = ee_target.x;
    double y = ee_target.y;
    double yaw = ee_target.yaw;

    double guess_elbow_joint = q_guess.at(1);

    // Trick: solve wrist first
    double x_wrist = x - L3 * std::cos(yaw);
    double y_wrist = y - L3 * std::sin(yaw);

    double arccos_input = (x_wrist*x_wrist + y_wrist*y_wrist - L1*L1 - L2*L2) / (2 * L1 * L2);

    // ---- FAILURE CHECK: OUT OF REACH ----
    // If the required geometry demands a triangle edge longer than physically possible, 
    // the arccos input will fall outside [-1, 1]. We allow a tiny epsilon for floating point math.
    if (arccos_input > 1.0 + 1e-6 || arccos_input < -1.0 - 1e-6) {
        q_solution = original_q_solution; // Behavior on failure: q_solution passes through unchanged.
        status = IKStatus::OUT_OF_REACH;
        return false;
    }

    // If the input to arccos is beyond -1.0 to 1.0, std::acos will return NaN, possibly causing issues.
    // It should already be safe by now, but if arccos_input was 1.000000000001 that could cause an issue, so we clamp it here.
    arccos_input = std::clamp(arccos_input, -1.0, 1.0); 

    // arc cosine returns two values. std::acos returns the principal value (0 to pi). The second solution is given by -std::acos
    double theta_2_sol1 = std::acos(arccos_input); // Elbow down
    double theta_2_sol2 = -std::acos(arccos_input); // Elbow up

    double theta_2{0.0};
    if (std::abs(wrap_to_pi(theta_2_sol1 - guess_elbow_joint)) < std::abs(wrap_to_pi(theta_2_sol2 - guess_elbow_joint))) {
        theta_2 = theta_2_sol1;
    }
    else {
        theta_2 = theta_2_sol2;
    }

    // Solve for theta_1 using theta_2
    double theta_1 = std::atan2(y_wrist, x_wrist) - std::atan2(L2*std::sin(theta_2), L1+L2*std::cos(theta_2));

    // Solve for theta_3
    double theta_3 = yaw - theta_1 - theta_2;


    // Strictly normalize all angles between [-pi, pi] just in case
    double t1_wrapped = wrap_to_pi(theta_1);
    double t2_wrapped = wrap_to_pi(theta_2);
    double t3_wrapped = wrap_to_pi(theta_3);

    // ---- FAILURE CHECK: JOINT LIMITS ----
    // Wrap angles between [-pi, pi] and check these fall within robot joint limits
    if (t1_wrapped < config_.joint_limits.at(0).at(0) || t1_wrapped > config_.joint_limits.at(0).at(1) ||
        t2_wrapped < config_.joint_limits.at(1).at(0) || t2_wrapped > config_.joint_limits.at(1).at(1) ||
        t3_wrapped < config_.joint_limits.at(2).at(0) || t3_wrapped > config_.joint_limits.at(2).at(1)) {
        
        q_solution = original_q_solution; // Behavior on failure: pass through untouched
        status = IKStatus::JOINT_LIMIT_VIOLATION;
        return false;
    }


    // ---- SUCCESS: Populate output ----
    q_solution.at(0) = t1_wrapped;
    q_solution.at(1) = t2_wrapped;
    q_solution.at(2) = t3_wrapped;

    status = IKStatus::SUCCESS;
    return true;
}



} // namespace planar_arm