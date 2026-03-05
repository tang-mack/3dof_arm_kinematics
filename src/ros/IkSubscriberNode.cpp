#include "planar_arm_kinematics/ros/IkSubscriberNode.h"
#include "planar_arm_kinematics/core/RobotModel.h"
#include "planar_arm_kinematics/core/Types.h"

#include "planar_arm_kinematics/core/AnalyticalSolver.h"
#include "planar_arm_kinematics/core/PinocchioSolver.h"

namespace planar_arm {

IkSubscriberNode::IkSubscriberNode() : Node("ik_subscriber_node") {
    this->declare_parameter<std::string>("yaml_filepath", "");
    this->declare_parameter<std::string>("solver_backend", "AnalyticalSolver"); // default to AnalyticalSolver (most robust so far)

    std::string yaml_filepath = this->get_parameter("yaml_filepath").as_string();
    std::string solver_backend = this->get_parameter("solver_backend").as_string();

    // Instantiate the correct solver at runtime
    if (solver_backend == "AnalyticalSolver") {
        RCLCPP_INFO(this->get_logger(), "Loading Analytical Solver");
        solver_ = std::make_unique<AnalyticalSolver>(yaml_filepath);
    }
    else if (solver_backend == "PinocchioSolver") {
        RCLCPP_INFO(this->get_logger(), "Loading Pinocchio Solver");
        solver_ = std::make_unique<PinocchioSolver>(yaml_filepath);
    }
    else {
        throw std::runtime_error("Unknown solver_backend: " + solver_backend);
    }

    // buffer queue of 10, drop oldest message (rationale: we care about recent data for IK)
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "end_effector_pose", 10, std::bind(&IkSubscriberNode::pose_callback, this, std::placeholders::_1));
}

void IkSubscriberNode::pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    // Convert ROS message back into our Pose_XY_Yaw struct
    Pose_XY_Yaw target_pose{msg->x, msg->y, msg->theta};

    // Call Inverse Kinematics
    IKStatus status;
    JointAnglesRad q_solution = {0.0, 0.0, 0.0};
    bool ik_ok = solver_->inverse_kinematics(target_pose, q_solution, previous_joints_, status);

    if (ik_ok == true) {
        previous_joints_ = q_solution; // Update previous joints

        // Print the result
        RCLCPP_INFO(this->get_logger(), "IK Node Received Pose. Output Angles -> t1: %.2f, t2: %.2f, t3: %.2f", 
        q_solution[0], q_solution[1], q_solution[2]);
    }
    else {
        // Handle the error
        // Throttle printing (1000ms) to prevent console spam: evaluate the failure every tick, but only print the warning to console once per second.
        auto clock = this->get_clock();
        
        switch (status) {
            case IKStatus::OUT_OF_REACH:
                RCLCPP_WARN_THROTTLE(this->get_logger(), *clock, 1000, 
                    "IK Failed: Target (%.2f, %.2f) is out of physical reach.", target_pose.x, target_pose.y);
                break;
            case IKStatus::MAX_ITERATIONS_REACHED:
                RCLCPP_WARN_THROTTLE(this->get_logger(), *clock, 1000, 
                    "IK Failed: Solver timed out (trapped in local minimum).");
                break;
            case IKStatus::OTHER_ERROR:
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *clock, 1000, 
                    "IK Failed: Other error.");
                break;
            default:
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *clock, 1000, 
                    "IK Failed: Unknown error.");
                break;
        }
    }

}

} // namespace planar_arm