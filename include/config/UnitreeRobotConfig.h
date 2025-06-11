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

    // ✅ 仅保留 custom 字段（属于该机器人独有）
    bool on_rack;
    int homing_timesteps;
    Eigen::VectorXf homingPos;
    Eigen::VectorXf homingKp;
    Eigen::VectorXf homingKd;
    
private:
    YAML::Node root_;
};
