#pragma once

class RobotModel {
public:
    RobotModel() = delete;
    explicit RobotModel(double l1, double l2, double l3) : L1(l1), L2(l2), L3(l3) {

    }

    const double L1;
    const double L2;
    const double L3;
};