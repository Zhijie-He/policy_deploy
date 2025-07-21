// ===== include/config/BaseRobotConfig.h =====
#pragma once

#include <Eigen/Dense>
#include <string>

class BaseRobotConfig {
public:
    virtual ~BaseRobotConfig() = default;
    int num_actions;
    int num_obs;
    int num_hidden;
    float simulation_dt;
    float policy_dt;
    int control_decimation;
    int domain_id;
    int print_interval;

    Eigen::VectorXf kP;
    Eigen::VectorXf kD;
    Eigen::VectorXf default_angles;
    Eigen::VectorXf effort_limit;
    Eigen::Vector3f cmd_init;
    Eigen::Vector3f cmd_scale;
    Eigen::Vector3f max_cmd;

    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float action_scale;

    std::string policy_path;
    std::string engine_path;
    std::string xml_path;
    std::string robot_name;
    virtual float getPolicyDt() const = 0;

    // eman
    std::string msg_type;
    std::string imu_type;
    std::string lowcmd_topic;
    std::string lowstate_topic;
    std::string imu_topic;
    std::string motor_topic;
    
    float obs_scale_projected_gravity_b;
    std::vector<std::string> actor_joint_names;
    std::vector<std::string> env_joint_names;
    std::vector<int> actor2env;
    std::vector<int> env2actor;

};
