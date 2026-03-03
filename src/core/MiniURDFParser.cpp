#include "planar_arm_kinematics/core/MiniURDFParser.h"
#include <stdexcept>
#include <sstream>
#include <cmath>
#include <iostream>

using namespace tinyxml2;

namespace planar_arm {

std::vector<double> MiniURDFParser::parse_link_lengths(const std::string& urdf_filepath)
{
    XMLDocument doc;
    if (doc.LoadFile(urdf_filepath.c_str()) != XML_SUCCESS) {
        throw std::runtime_error("[MiniURDFParser] Failed to load URDF file, path provided was " + urdf_filepath);
    }

    XMLElement* robot = doc.FirstChildElement("robot");
    if (!robot) {
        throw std::runtime_error("[MiniURDFParser] No <robot> element found.");
    }

    std::vector<double> lengths;

    for (XMLElement* joint = robot->FirstChildElement("joint");
         joint != nullptr;
         joint = joint->NextSiblingElement("joint"))
    {
        XMLElement* origin = joint->FirstChildElement("origin");
        if (!origin) continue;

        const char* xyz_str = origin->Attribute("xyz");
        if (!xyz_str) continue;

        std::istringstream iss(xyz_str);
        double x = 0.0, y = 0.0, z = 0.0;
        iss >> x >> y >> z;

        // Calculate the full 3D length
        double length = std::sqrt(x*x + y*y + z*z);

        // Ignore dummy joints (like world_to_base) that have no translation
        if (length > 1e-4) {
            lengths.push_back(length);
        }
    }

    // Unconditionally drop the first valid length found (assuming it is the base standoff)
    if (!lengths.empty()) {
        std::cout << "[MiniURDFParser] Skipping base standoff length: " << lengths.front() << std::endl;
        lengths.erase(lengths.begin());
    }

    if (lengths.size() < 3) {
        throw std::runtime_error("[MiniURDFParser] Expected at least 3 link segments after skipping the base standoff.");
    }

    // Restrict to exactly 3 links
    lengths.resize(3);

    return lengths;
}

} // namespace planar_arm