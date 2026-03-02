#pragma once

#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace planar_arm {

class YamlReader {
public:
    YamlReader() = default;

    bool load(const std::string& filepath) {
        try {
            root_node_ = YAML::LoadFile(filepath);
            return true;
        } catch (const YAML::Exception& e) {
            std::cerr << "[YamlReader] Failed to load config file, yaml not found at: " << filepath 
                      << " | Error: " << e.what() << std::endl;
            return false;
        }
    }

    // Fetches top-level yaml keys (just the "root level" content in the yaml file without any indentation)
    template <typename T>
    T get_global(const std::string& key) const {
        if (root_node_[key]) {
            return root_node_[key].as<T>();
        }
        throw std::runtime_error("[YamlReader] Missing required global YAML key: " + key);
    }

    // Fetches nested yaml keys, for example if CustomSolver is at root level, and we need indented values (e.g., CustomSolver -> use_lookup_table_speedup)
    template <typename T>
    T get_local(const std::string& class_namespace, const std::string& key) const {
        if (root_node_[class_namespace] && root_node_[class_namespace][key]) {
            return root_node_[class_namespace][key].as<T>();
        }
        throw std::runtime_error("[YamlReader] Missing required local YAML key: " + class_namespace + " -> " + key);
    }

private:
    YAML::Node root_node_;
};

} // namespace planar_arm