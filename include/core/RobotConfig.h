// ========== include/core/RobotConfig.h ==========
#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <string>

class RobotConfig {
public:
    explicit RobotConfig(const std::string& yaml_path);
    const YAML::Node& raw() const;
    float getPolicyDt() const; 

    int num_actions;
    int num_obs;
    float simulation_duration;
    float simulation_dt;
    int control_decimation;

    Eigen::VectorXf kP;
    Eigen::VectorXf kD;
    Eigen::VectorXf default_angles;
    Eigen::Vector3f cmd_scale;
    Eigen::Vector3f cmd_init;

    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float action_scale;

    std::string policy_path;
    std::string xml_path;

    // custom
    std::string robot_name;
    bool on_rack;
    std::string world_type;
    std::string urdf_path;
    int homing_timesteps;
    Eigen::VectorXf homingPos;
    Eigen::VectorXf homingKp;
    Eigen::VectorXf homingKd;
    Eigen::VectorXf init_base_position;
    Eigen::VectorXf init_base_orientation;

private:
    YAML::Node root_;
};
