// ========== src/config/EmanRobotConfig.cpp ==========
#include "config/EmanRobotConfig.h"
#include "utility/logger.h"

inline std::string get_optional_string(const YAML::Node& node, const std::string& key, const std::string& default_val = "") {
    return (node[key] && node[key].IsScalar()) ? node[key].as<std::string>() : default_val;
}

inline HandConfig loadHandConfigFromNode(const YAML::Node& node) {
    HandConfig config;

    const auto& kpl = node["kps"]["left"].as<std::vector<float>>();
    const auto& kpr = node["kps"]["right"].as<std::vector<float>>();
    const auto& kdl = node["kds"]["left"].as<std::vector<float>>();
    const auto& kdr = node["kds"]["right"].as<std::vector<float>>();

    config.kp_left  = Eigen::Map<const Eigen::VectorXf>(kpl.data(), kpl.size());
    config.kp_right = Eigen::Map<const Eigen::VectorXf>(kpr.data(), kpr.size());
    config.kd_left  = Eigen::Map<const Eigen::VectorXf>(kdl.data(), kdl.size());
    config.kd_right = Eigen::Map<const Eigen::VectorXf>(kdr.data(), kdr.size());

    config.left_cmd_topic    = get_optional_string(node, "left_lowcmd_topic");
    config.right_cmd_topic   = get_optional_string(node, "right_lowcmd_topic");
    config.left_state_topic  = get_optional_string(node, "left_lowstate_topic");
    config.right_state_topic = get_optional_string(node, "right_lowstate_topic");

    // add hands logical
    config.hands_num = config.kp_left.size() + config.kp_right.size();
    config.left_hand_num_dof = config.kp_left.size();
    config.right_hand_num_dof = config.kp_right.size();

    int size = 6 + 6 + 3 + 7 + config.left_hand_num_dof  + 7 + config.right_hand_num_dof;
    config.joint_concat_index.resize(size);
    int idx = 0;
    for (int i = 0; i < 6; ++i) config.joint_concat_index(idx++) = i;   // [0,6)
    for (int i = 6; i < 12; ++i) config.joint_concat_index(idx++) = i;  // [6,12)
    for (int i = 12; i < 15; ++i) config.joint_concat_index(idx++) = i; // [12,15)
    for (int i = 15; i < 22; ++i) config.joint_concat_index(idx++) = i; // [15,22)
    for (int i = 29; i < 29 + config.left_hand_num_dof ; ++i) config.joint_concat_index(idx++) = i;   // [29,29+L)
    for (int i = 22; i < 29; ++i) config.joint_concat_index(idx++) = i;   // [22,29)
    for (int i = 29 + config.left_hand_num_dof ; i < 29 + config.left_hand_num_dof  + config.right_hand_num_dof; ++i)   // [29+L,29+L+R)
        config.joint_concat_index(idx++) = i;
    
    // FRC_CRITICAL("joint_concat_index" << config.joint_concat_index.transpose());

    // 构建 skeleton 下标
    std::vector<int> skeleton_vec;
    for (int i = 0; i < 6; ++i) skeleton_vec.push_back(i);
    for (int i = 6; i < 12; ++i) skeleton_vec.push_back(i);
    for (int i = 12; i < 15; ++i) skeleton_vec.push_back(i);
    for (int i = 15; i < 22; ++i) skeleton_vec.push_back(i);
    for (int i = 22 + config.left_hand_num_dof; i < 22 + config.left_hand_num_dof + 7; ++i)
        skeleton_vec.push_back(i);
    
    // 构建 hands 下标
    std::vector<int> hands_vec;
    for (int i = 22; i < 22 + config.left_hand_num_dof; ++i)
        hands_vec.push_back(i);
    for (int i = 22 + config.left_hand_num_dof + 7; i < 22 + config.left_hand_num_dof + 7 + config.right_hand_num_dof; ++i)
        hands_vec.push_back(i);


    // 转为 Eigen::VectorXi
    config.joint_split_index["skeleton"] = Eigen::Map<Eigen::VectorXi>(skeleton_vec.data(), skeleton_vec.size());
    config.joint_split_index["hands"]    = Eigen::Map<Eigen::VectorXi>(hands_vec.data(), hands_vec.size());

    return config;  
}

EmanRobotConfig::EmanRobotConfig(const std::string& yaml_path)
    : root_(YAML::LoadFile(yaml_path))  // 初始化 YAML 配置节点
{   
    // ========== 基础参数 ==========
    num_actions = root_["num_actions"].as<int>();
    simulation_dt = root_["simulation_dt"].as<float>();
    control_decimation = root_["control_decimation"].as<int>();
    policy_dt = simulation_dt *  control_decimation;
    
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
    const auto& max_cmd_vec  = root_["max_cmd"].as<std::vector<float>>();
    cmd_scale = Eigen::Map<const Eigen::Vector3f>(cmd_scale_vec.data());
    cmd_init  = Eigen::Map<const Eigen::Vector3f>(cmd_init_vec.data());
    max_cmd  = Eigen::Map<const Eigen::Vector3f>(max_cmd_vec.data());
    
    ang_vel_scale = root_["obs_scale_root_ang_vel_b"].as<float>();
    dof_pos_scale = root_["obs_scale_joint_pos"].as<float>();
    dof_vel_scale = root_["obs_scale_joint_vel"].as<float>();
    action_scale  = root_["action_scale"].as<float>();

    // ========== 路径信息 ==========
    std::string project_source_dir = PROJECT_SOURCE_DIR;
    xml_path = project_source_dir + "/" + root_["xml_path"].as<std::string>();
    xml_with_hand_path = project_source_dir + "/" + root_["xml_with_hand_path"].as<std::string>();
    // ========== 自定义参数 ==========
    robot_name = root_["robot_name"].as<std::string>();
    
    msg_type = root_["msg_type"].as<std::string>();
    imu_type = root_["imu_type"].as<std::string>();
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

    // get hands config
    // 在构造函数中
    const YAML::Node& hands_node = root_["hands"];
    for (const auto& it : hands_node) {
        const std::string name = it.first.as<std::string>();
        const auto& node = it.second;
        hand_map[name] = loadHandConfigFromNode(node);
    }
}



const YAML::Node& EmanRobotConfig::raw() const {
    return root_;
}

float EmanRobotConfig::getPolicyDt() const {
    return simulation_dt * control_decimation;
}
