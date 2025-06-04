// ========== src/core/RobotConfig.cpp ==========
#include "core/RobotConfig.h"

RobotConfig::RobotConfig(const std::string& yaml_path)
    : root_(YAML::LoadFile(yaml_path))  // 初始化 YAML 配置节点
{
    // ========== 基础参数 ==========
    num_actions = root_["num_actions"].as<int>();
    num_obs = root_["num_obs"].as<int>();
    simulation_dt = root_["simulation_dt"].as<float>();
    control_decimation = root_["control_decimation"].as<int>();

    // ========== 控制增益 ==========
    const auto& kps = root_["kps"].as<std::vector<float>>();
    const auto& kds = root_["kds"].as<std::vector<float>>();
    kP = Eigen::Map<const Eigen::VectorXf>(kps.data(), kps.size());
    kD = Eigen::Map<const Eigen::VectorXf>(kds.data(), kds.size());

    // ========== 默认关节角 ==========
    const auto& angles = root_["default_angles"].as<std::vector<float>>();
    default_angles = Eigen::Map<const Eigen::VectorXf>(angles.data(), angles.size());

    // ========== 命令缩放与初始值 ==========
    const auto& cmd_scale_vec = root_["cmd_scale"].as<std::vector<float>>();
    const auto& cmd_init_vec  = root_["cmd_init"].as<std::vector<float>>();
    cmd_scale = Eigen::Map<const Eigen::Vector3f>(cmd_scale_vec.data());
    cmd_init  = Eigen::Map<const Eigen::Vector3f>(cmd_init_vec.data());

    ang_vel_scale = root_["ang_vel_scale"].as<float>();
    dof_pos_scale = root_["dof_pos_scale"].as<float>();
    dof_vel_scale = root_["dof_vel_scale"].as<float>();
    action_scale  = root_["action_scale"].as<float>();

    // ========== 路径信息 ==========
    std::string project_source_dir = PROJECT_SOURCE_DIR;
    policy_path = project_source_dir + "/" + root_["policy_path"].as<std::string>();
    xml_path    = project_source_dir + "/" + root_["xml_path"].as<std::string>();

    // ========== 自定义参数 ==========
    robot_name = root_["robot_name"].as<std::string>();
    on_rack = root_["on_rack"] ? root_["on_rack"].as<bool>() : false;
    world_type = root_["world_type"].as<std::string>();
    urdf_path =  project_source_dir + "/" + root_["urdf_path"].as<std::string>();
    homing_timesteps = root_["homing_timesteps"].as<int>();

    // ========== Homing 参数（可选）==========
    if (root_["homing"]) {
        const auto& pos = root_["homing"]["pos"].as<std::vector<float>>();
        const auto& kp  = root_["homing"]["kp"].as<std::vector<float>>();
        const auto& kd  = root_["homing"]["kd"].as<std::vector<float>>();
        homingPos = Eigen::Map<const Eigen::VectorXf>(pos.data(), pos.size());
        homingKp  = Eigen::Map<const Eigen::VectorXf>(kp.data(), kp.size());
        homingKd  = Eigen::Map<const Eigen::VectorXf>(kd.data(), kd.size());
    }
    terrain_config_file    = project_source_dir + "/" + root_["terrain_config_file"].as<std::string>();
}

const YAML::Node& RobotConfig::raw() const {
    return root_;
}

float RobotConfig::getPolicyDt() const {
    return simulation_dt * control_decimation;
}
