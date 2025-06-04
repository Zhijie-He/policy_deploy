// ===== include/core/BaseRobotConfig.h =====
#pragma once

#include <Eigen/Dense>
#include <string>

class BaseRobotConfig {
public:
    virtual ~BaseRobotConfig() = default;

    int num_actions;
    int num_obs;
    float simulation_dt;
    int control_decimation;

    Eigen::VectorXf kP;
    Eigen::VectorXf kD;
    Eigen::VectorXf default_angles;
    Eigen::Vector3f cmd_init;
    Eigen::Vector3f cmd_scale;

    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float action_scale;

    std::string policy_path;
    std::string xml_path;
    std::string robot_name;
    
    virtual float getPolicyDt() const = 0;

    float obs_scale_projected_gravity_b;
    std::vector<std::string> actor_joint_names;
    std::vector<std::string> env_joint_names;
    std::vector<int> actor2env;
    std::vector<int> env2actor;
    
};
