#pragma once

#include <tinyxml2.h>
#include <vector>
#include <string>

namespace planar_arm {

class MiniURDFParser {
public:
    static std::vector<double> parse_link_lengths(const std::string& urdf_filepath);

private:
    static double extract_length_from_joint(tinyxml2::XMLElement* joint_elem);
};

}