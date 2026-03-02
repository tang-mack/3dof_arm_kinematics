#pragma once

#include <array>

namespace planar_arm {

    // --- Define Data Types ---
    struct Pose_XY_Yaw {
        double x{0.0};   // [m] x-position with respect to world frame
        double y{0.0};   // [m] y-position with respect to world frame
        double yaw{0.0}; // [rad] Yaw about world frame z-axis
    };

    using JointAnglesRad = std::array<double, 3>; // [rad] Joint angles, theta_1, theta_2, theta_3
    
} // namespace planar_arm