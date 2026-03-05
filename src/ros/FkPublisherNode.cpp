#include "planar_arm_kinematics/ros/FkPublisherNode.h"
#include "planar_arm_kinematics/core/RobotModel.h"
#include <chrono>
#include "planar_arm_kinematics/core/AnalyticalSolver.h"
#include "planar_arm_kinematics/core/PinocchioSolver.h"

using namespace std::chrono_literals;

namespace planar_arm {

FkPublisherNode::FkPublisherNode() : Node("fk_publisher_node") {

    this->declare_parameter<std::string>("yaml_filepath", "");
    this->declare_parameter<std::string>("solver_backend", "AnalyticalSolver"); // default to AnalyticalSolver (most robust so far)

    // Declare the launch file parameters
    this->declare_parameter("theta1", 0.0);
    this->declare_parameter("theta2", 0.0);
    this->declare_parameter("theta3", 0.0);

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

    // Publisher setup
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("end_effector_pose", 10);
    
    // Publish at 1 Hz
    timer_ = this->create_wall_timer(1s, std::bind(&FkPublisherNode::timer_callback, this));
}

void FkPublisherNode::timer_callback() {
    // Fetch the latest parameters
    double t1 = this->get_parameter("theta1").as_double();
    double t2 = this->get_parameter("theta2").as_double();
    double t3 = this->get_parameter("theta3").as_double();

    // Call our pure non-ROS C++ API
    JointAnglesRad q_current = {t1, t2, t3};
    Pose_XY_Yaw pose = solver_->forward_kinematics(q_current);

    // Convert to a ROS message and publish
    auto msg = geometry_msgs::msg::Pose2D();
    msg.x = pose.x;
    msg.y = pose.y;
    msg.theta = pose.yaw;

    RCLCPP_INFO(this->get_logger(), "FK Node Publishing -> x: %.2f, y: %.2f, yaw: %.2f", msg.x, msg.y, msg.theta);
    publisher_->publish(msg);
}

} // namespace planar_arm