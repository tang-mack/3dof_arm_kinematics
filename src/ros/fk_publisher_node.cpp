#include "planar_arm_kinematics/ros/fk_publisher_node.h"
#include <chrono>

using namespace std::chrono_literals;

namespace planar_arm {

FkPublisherNode::FkPublisherNode() : Node("fk_publisher_node"), 
    // Initialize the backend solver with default link lengths (L1=0.3, L2=0.3, L3=0.1)
    kinematics_(CustomSolver(0.3, 0.3, 0.1)) 
{
    // Declare the launch file parameters
    this->declare_parameter("theta1", 0.0);
    this->declare_parameter("theta2", 0.0);
    this->declare_parameter("theta3", 0.0);

    publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("end_effector_pose", 10);
    
    // Publish at 1 Hz
    timer_ = this->create_wall_timer(1s, std::bind(&FkPublisherNode::timer_callback, this));
}

void FkPublisherNode::timer_callback() {
    // 1. Fetch the latest parameters
    double t1 = this->get_parameter("theta1").as_double();
    double t2 = this->get_parameter("theta2").as_double();
    double t3 = this->get_parameter("theta3").as_double();

    // 2. Call our pure non-ROS C++ API
    JointAnglesRad joints = {t1, t2, t3};
    Pose_XY_Yaw pose = kinematics_.compute_fk(joints);

    // 3. Convert to a ROS message and publish
    auto msg = geometry_msgs::msg::Pose2D();
    msg.x = pose.x;
    msg.y = pose.y;
    msg.theta = pose.yaw;

    RCLCPP_INFO(this->get_logger(), "FK Node Publishing -> x: %.2f, y: %.2f, yaw: %.2f", msg.x, msg.y, msg.theta);
    publisher_->publish(msg);
}

} // namespace planar_arm