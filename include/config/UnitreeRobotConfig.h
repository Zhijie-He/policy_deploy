// ========== include/config/UnitreeRobotConfig.h ==========
#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <string>
#include "config/BaseRobotConfig.h"

class UnitreeRobotConfig : public BaseRobotConfig {
public:
    explicit UnitreeRobotConfig(const std::string& yaml_path);
    const YAML::Node& raw() const;
    float getPolicyDt() const override;
private:
    YAML::Node root_;
};
