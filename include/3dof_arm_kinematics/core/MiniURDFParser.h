#pragma once

#include <tinyxml2.h>
#include <vector>
#include <string>

namespace arm_3dof {

class MiniURDFParser {
public:
    /// @brief Take a look at the URDF, return a vector of link lengths 
    static std::vector<double> parse_link_lengths(const std::string& urdf_filepath);

private:
    static double extract_length_from_joint(tinyxml2::XMLElement* joint_elem);
};

}