// ===== include/config/BaseRobotConfig.h =====
#pragma once

#include <string>
#include <unordered_map>
#include <Eigen/Dense>

struct HandConfig {
    Eigen::VectorXf kp_left, kp_right;
    Eigen::VectorXf kd_left, kd_right;

    std::string left_cmd_topic = "";
    std::string right_cmd_topic = "";
    std::string left_state_topic = "";
    std::string right_state_topic = "";

    // custom hands related
    int hands_num = 0;
    int left_hand_num_dof = 0;
    int right_hand_num_dof = 0;
    Eigen::VectorXi joint_concat_index;
    std::unordered_map<std::string, Eigen::VectorXi> joint_split_index;
};


class BaseRobotConfig {
public:
    virtual ~BaseRobotConfig() = default;
    int num_actions;
    float simulation_dt;
    float policy_dt;
    int control_decimation;

    Eigen::VectorXf kP;
    Eigen::VectorXf kD;
    Eigen::VectorXf default_angles;
    Eigen::Vector3f cmd_init;
    Eigen::Vector3f cmd_scale;
    Eigen::Vector3f max_cmd;

    float ang_vel_scale;
    float dof_pos_scale;
    float dof_vel_scale;
    float action_scale;
    
    std::string xml_path;
    std::string xml_with_hand_path;
    std::string robot_name;
    virtual float getPolicyDt() const = 0;

    // eman
    std::string msg_type;
    std::string imu_type;
    std::string lowcmd_topic;
    std::string lowstate_topic;
    float obs_scale_projected_gravity_b;
    std::vector<std::string> actor_joint_names;
    std::vector<std::string> env_joint_names;
    std::vector<int> actor2env;
    std::vector<int> env2actor;

    // hands
    std::unordered_map<std::string, HandConfig> hand_map;
};
