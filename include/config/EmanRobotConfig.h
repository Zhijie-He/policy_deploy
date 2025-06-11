// ========== include/config/EmanRobotConfig.h ==========
#pragma once
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <string>
#include "config/BaseRobotConfig.h"

class EmanRobotConfig : public BaseRobotConfig {
public:
    explicit EmanRobotConfig(const std::string& yaml_path);
    const YAML::Node& raw() const;
    float getPolicyDt() const override;
    
private:
    YAML::Node root_;
};
