#include "planar_arm_kinematics/ros/ik_subscriber_node.h"

namespace planar_arm {

IkSubscriberNode::IkSubscriberNode() : Node("ik_subscriber_node"),
    kinematics_(CustomSolver(0.3, 0.3, 0.1)) 
{
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "end_effector_pose", 10, std::bind(&IkSubscriberNode::pose_callback, this, std::placeholders::_1));
}

void IkSubscriberNode::pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    // 1. Convert ROS message back into our custom C++ struct
    Pose_XY_Yaw target_pose{msg->x, msg->y, msg->theta};

    // 2. Call our pure Non ROS C++ API
    JointAnglesRad computed_joints = kinematics_.compute_ik(target_pose);

    // 3. Mock override as requested
    computed_joints = {-99.0, -99.0, -99.0};

    // 4. Print the result
    RCLCPP_INFO(this->get_logger(), "IK Node Received Pose. Mocked Angles -> t1: %.2f, t2: %.2f, t3: %.2f", 
                computed_joints[0], computed_joints[1], computed_joints[2]);
}

} // namespace planar_arm