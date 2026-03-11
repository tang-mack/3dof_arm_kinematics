#pragma once

#include <vector>
#include <stdexcept>
#include <iostream>

class RobotModel {
public:
    RobotModel() = default; // Rationale: the RobotModel is simply empty initially, no garbage data.

    explicit RobotModel(const std::vector<double>& lengths)
    {
        if (lengths.empty()) {
            throw std::runtime_error("[RobotModel] Cannot construct with zero link lengths.");
        }

        link_lengths_.resize(lengths.size());
        std::copy(lengths.begin(), lengths.end(), link_lengths_.begin());
    }

    std::size_t n_dofs() const {
        return link_lengths_.size();
    }

    /// @brief Function for zero-indexed get link length in meters.
    /// @return Returns link length for L1, L2, L3 etc.
    double get_length(std::size_t i) const {
        return link_lengths_.at(i);
    }

    void print_link_lengths() {
        for (const auto& ll : link_lengths_) {
            std::cout << "link_length: " << ll << std::endl;
        }
    }

private:
    std::vector<double> link_lengths_;
};