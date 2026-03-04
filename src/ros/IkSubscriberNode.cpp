#include "planar_arm_kinematics/ros/IkSubscriberNode.h"
#include "planar_arm_kinematics/core/RobotModel.h"

namespace planar_arm {

IkSubscriberNode::IkSubscriberNode() : Node("ik_subscriber_node"),
    kinematics_(ActiveSolverBackend(this->declare_parameter<std::string>("yaml_filepath", ""))) 
{
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "end_effector_pose", 10, std::bind(&IkSubscriberNode::pose_callback, this, std::placeholders::_1));
}

void IkSubscriberNode::pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    // Convert ROS message back into our custom C++ struct
    Pose_XY_Yaw target_pose{msg->x, msg->y, msg->theta};

    // Call pure Non ROS C++ API
    // JointAnglesRad computed_joints = kinematics_.compute_ik(target_pose, previous_joints_[1]); // guess with previous elbow
    IKStatus status;
    JointAnglesRad q_solution = {0.0, 0.0, 0.0};
    bool ik_ok = kinematics_.compute_ik(target_pose, q_solution, previous_joints_, status);

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