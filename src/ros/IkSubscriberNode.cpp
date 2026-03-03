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
    JointAnglesRad computed_joints = kinematics_.compute_ik(target_pose, previous_joints_[1]); // guess with previous elbow

    previous_joints_ = computed_joints; // Update previous joints

    // Print the result
    RCLCPP_INFO(this->get_logger(), "IK Node Received Pose. Output Angles -> t1: %.2f, t2: %.2f, t3: %.2f", 
                computed_joints[0], computed_joints[1], computed_joints[2]);
}

} // namespace planar_arm