// ========== src/core/EmanRobotConfig.cpp ==========
#include "core/EmanRobotConfig.h"
#include "utility/logger.h"

EmanRobotConfig::EmanRobotConfig(const std::string& yaml_path)
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
    const auto& angles = root_["default_joint_pos"].as<std::vector<float>>();
    default_angles = Eigen::Map<const Eigen::VectorXf>(angles.data(), angles.size());

    // ========== 命令缩放与初始值 ==========
    const auto& cmd_scale_vec = root_["obs_scale_cmd"].as<std::vector<float>>();
    const auto& cmd_init_vec  = root_["cmd_init"].as<std::vector<float>>();
    cmd_scale = Eigen::Map<const Eigen::Vector3f>(cmd_scale_vec.data());
    cmd_init  = Eigen::Map<const Eigen::Vector3f>(cmd_init_vec.data());

    ang_vel_scale = root_["obs_scale_root_ang_vel_b"].as<float>();
    dof_pos_scale = root_["obs_scale_joint_pos"].as<float>();
    dof_vel_scale = root_["obs_scale_joint_vel"].as<float>();
    action_scale  = root_["action_scale"].as<float>();

    // ========== 路径信息 ==========
    std::string project_source_dir = PROJECT_SOURCE_DIR;
    policy_path = project_source_dir + "/" + root_["policy_path"].as<std::string>();
    xml_path    = project_source_dir + "/" + root_["xml_path"].as<std::string>();

    // ========== 自定义参数 ==========
    robot_name = root_["robot_name"].as<std::string>();
    msg_type = root_["msg_type"].as<std::string>();
    lowcmd_topic = root_["lowcmd_topic"].as<std::string>();
    lowstate_topic = root_["lowstate_topic"].as<std::string>();
    obs_scale_projected_gravity_b = root_["obs_scale_projected_gravity_b"].as<float>();
    actor_joint_names = root_["actor_joint_names"].as<std::vector<std::string>>();
    env_joint_names = root_["env_joint_names"].as<std::vector<std::string>>();

    // === 构建 actor2env 映射 ===
    actor2env.resize(env_joint_names.size());
    for (size_t i = 0; i < env_joint_names.size(); ++i) {
        const auto& name = env_joint_names[i];
        auto it = std::find(actor_joint_names.begin(), actor_joint_names.end(), name);
        if (it == actor_joint_names.end()) {
            throw std::runtime_error("[EmanRobotConfig] env_joint '" + name + "' not found in actor_joint_names.");
        }
        actor2env[i] = static_cast<int>(std::distance(actor_joint_names.begin(), it));
    }

    // === 构建 env2actor 映射 ===
    env2actor.resize(actor_joint_names.size());
    for (size_t i = 0; i < actor_joint_names.size(); ++i) {
        const auto& name = actor_joint_names[i];
        auto it = std::find(env_joint_names.begin(), env_joint_names.end(), name);
        if (it == env_joint_names.end()) {
            throw std::runtime_error("[EmanRobotConfig] actor_joint '" + name + "' not found in env_joint_names.");
        }
        env2actor[i] = static_cast<int>(std::distance(env_joint_names.begin(), it));
    }
}

const YAML::Node& EmanRobotConfig::raw() const {
    return root_;
}

float EmanRobotConfig::getPolicyDt() const {
    return simulation_dt * control_decimation;
}
