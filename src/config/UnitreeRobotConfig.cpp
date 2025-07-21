// ========== src/config/UnitreeRobotConfig.cpp ==========
#include "config/UnitreeRobotConfig.h"
#include "utility/logger.h"

UnitreeRobotConfig::UnitreeRobotConfig(const std::string& yaml_path)
    : root_(YAML::LoadFile(yaml_path))  // 初始化 YAML 配置节点
{
    // ========== 基础参数 ==========
    num_actions = root_["num_actions"].as<int>();
    num_obs = root_["num_obs"].as<int>();
    num_hidden = root_["num_hidden"].as<int>();
    simulation_dt = root_["simulation_dt"].as<float>();
    control_decimation = root_["control_decimation"].as<int>();
    policy_dt = simulation_dt *  control_decimation;
    domain_id = root_["domain_id"].as<int>();
    
    // ========== 控制增益 ==========
    const auto& kps = root_["kps"].as<std::vector<float>>();
    const auto& kds = root_["kds"].as<std::vector<float>>();
    kP = Eigen::Map<const Eigen::VectorXf>(kps.data(), kps.size());
    kD = Eigen::Map<const Eigen::VectorXf>(kds.data(), kds.size());

    // ========== 默认关节角 ==========
    const auto& angles = root_["default_angles"].as<std::vector<float>>();
    default_angles = Eigen::Map<const Eigen::VectorXf>(angles.data(), angles.size());
    
    // ========== 力矩限制 ==========
    if (root_["effort_limit"] && root_["effort_limit"].IsSequence() && root_["effort_limit"].size() > 0) {
        const auto& effort_vec = root_["effort_limit"].as<std::vector<float>>();
        if(effort_vec.size() != num_actions ){
            FRC_INFO("[UnitreeRobotConfig.Const] Length mismatch between effort_limit and num_actions!");
            throw std::runtime_error("Length mismatch between effort_limit and num_actions!");
        }
        effort_limit = Eigen::Map<const Eigen::VectorXf>(effort_vec.data(), effort_vec.size());
    } else {
        FRC_WARN("[UnitreeRobotConfig.Const] 'effort_limit' not set or empty, using +inf.");
        effort_limit = Eigen::VectorXf::Constant(num_actions, std::numeric_limits<float>::infinity());
    }

    // ========== 命令缩放与初始值 ==========
    const auto& cmd_scale_vec = root_["cmd_scale"].as<std::vector<float>>();
    const auto& cmd_init_vec  = root_["cmd_init"].as<std::vector<float>>();
    const auto& max_cmd_vec  = root_["max_cmd"].as<std::vector<float>>();
    cmd_scale = Eigen::Map<const Eigen::Vector3f>(cmd_scale_vec.data());
    cmd_init  = Eigen::Map<const Eigen::Vector3f>(cmd_init_vec.data());
    max_cmd  = Eigen::Map<const Eigen::Vector3f>(max_cmd_vec.data());
    
    ang_vel_scale = root_["ang_vel_scale"].as<float>();
    dof_pos_scale = root_["dof_pos_scale"].as<float>();
    dof_vel_scale = root_["dof_vel_scale"].as<float>();
    action_scale  = root_["action_scale"].as<float>();

    // ========== 路径信息 ==========
    std::string project_source_dir = PROJECT_SOURCE_DIR;
    policy_path = project_source_dir + "/" + root_["policy_path"].as<std::string>();
    engine_path = project_source_dir + "/" + root_["engine_path"].as<std::string>();
    xml_path    = project_source_dir + "/" + root_["xml_path"].as<std::string>();

    // ========== 自定义参数 ==========
    robot_name = root_["robot_name"].as<std::string>();
    lowcmd_topic = root_["lowcmd_topic"].as<std::string>();
    lowstate_topic = root_["lowstate_topic"].as<std::string>();
}

const YAML::Node& UnitreeRobotConfig::raw() const {
    return root_;
}

float UnitreeRobotConfig::getPolicyDt() const {
    return simulation_dt * control_decimation;
}
