// tasks/MocapTask.cpp
#include "tasks/MocapTask.h"
#include "utility/logger.h"
#include "utility/tools.h"

MocapTask::MocapTask(std::shared_ptr<const BaseRobotConfig> cfg,
                     torch::Device device,
                     const std::string& hands_type,
                     const std::string& inference_engine_type,
                     const std::string& precision)
      : BaseTask(cfg, std::make_shared<MocapTaskCfg>(), device, hands_type, inference_engine_type, precision),
        task_cfg_() 
{
    FRC_INFO("[MocapTask.Const] MocapTask Created!");

    // overwrite cfg from model cfg
    // task_cfg_.num_samples = 1 + self.actor.cfg["task_next_obs"]["shape"][0]

    // 1. 提取 track_keypoints 对应的下标索引
    const auto& body_names = task_cfg_.BODY_NAMES;
    const auto& track_names = task_cfg_.track_keypoints_names;

    std::vector<int> temp_indices;
    for (const auto& name : track_names) {
        auto it = std::find(body_names.begin(), body_names.end(), name);
        if (it != body_names.end()) {
            temp_indices.push_back(static_cast<int>(std::distance(body_names.begin(), it)));
        } else {
            throw std::runtime_error("Body name not found in BODY_NAMES: " + name);
        }
    }

    // 转换为 Eigen::VectorXi
    track_keypoints_indices_ = Eigen::Map<Eigen::VectorXi>(temp_indices.data(), static_cast<int>(temp_indices.size()));

    // 2. 创建 mask_ 向量
    int n_bodies = static_cast<int>(body_names.size());
    mask_ = Eigen::VectorXf::Zero(n_bodies);
    for (int i = 0; i < track_keypoints_indices_.size(); ++i) {
        mask_[track_keypoints_indices_[i]] = 1.0f;
    }
    FRC_INFO("[MocapTask.Const] mask" << mask_.transpose());

    // 3. create mocap receiver
    mocap_receiver_ = std::make_unique<MocapMsgSubscriber>(task_cfg_.sample_timestep_inv, task_cfg_.num_samples);

    // 4. used for MocapTask with hands
    std::vector<int> temp_left;
    for (const auto& name : std::vector<std::string>{
        "left_wrist_roll_link",
        "left_wrist_pitch_link",
        "left_wrist_yaw_link"
    }) {
        auto it = std::find(body_names.begin(), body_names.end(), name);
        if (it != body_names.end()) {
            temp_left.push_back(static_cast<int>(std::distance(body_names.begin(), it)) - 1);
        } else {
            throw std::runtime_error("Missing body name: " + name);
        }
    }
    left_wrist_ids_ = Eigen::Map<Eigen::VectorXi>(temp_left.data(), static_cast<int>(temp_left.size()));

    // --- 初始化 right_wrist_ids_
    std::vector<int> temp_right;
    for (const auto& name : std::vector<std::string>{
        "right_wrist_roll_link",
        "right_wrist_pitch_link",
        "right_wrist_yaw_link"
    }) {
        auto it = std::find(body_names.begin(), body_names.end(), name);
        if (it != body_names.end()) {
            temp_right.push_back(static_cast<int>(std::distance(body_names.begin(), it)) - 1);
        } else {
            throw std::runtime_error("Missing body name: " + name);
        }
    }
    right_wrist_ids_ = Eigen::Map<Eigen::VectorXi>(temp_right.data(), static_cast<int>(temp_right.size()));
}

void MocapTask::resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) {
    FRC_INFO("[MocapTask.resolveKeyboardInput] Key pressed: " << key);
}

void MocapTask::resolveSelfObservation(const CustomTypes::RobotData& raw_obs) {
    BaseTask::resolveSelfObservation(raw_obs);
}

void MocapTask::resolveTaskObservation(const CustomTypes::RobotData& raw_obs) {
    int self_obs_len = 93;

    MocapResult mocap = getMocap();
    const auto& cache = mocap.teleop_obs;
    // const auto& points = mocap.motion_state;
    
    setVisualization(mocap.visualization);
  
    // 3. 构造 task_obs（当前帧）
    Eigen::VectorXf task_obs(cache[0].size());
    for (size_t i = 0; i < cache[0].size(); ++i) {
        task_obs[i] = cache[0][i];
    }

    // 4. 构造 task_next_obs（剩下 8 帧）
    Eigen::VectorXf task_next_obs((task_cfg_.num_samples - 1) * cache[0].size());
    for (int i = 1; i < task_cfg_.num_samples; ++i) {
        for (size_t j = 0; j < cache[i].size(); ++j) {
            task_next_obs[(i - 1) * cache[i].size() + j] = cache[i][j];
        }
    }

    // 5. heading
    float heading = tools::getHeadingFromQuat(raw_obs.root_rot);
    float scaled_heading = heading * task_cfg_.self_obs_scale.at("heading");

    // 6. 拼接 heading + task_obs
    Eigen::VectorXf final_task_obs(task_obs.size() + 1);
    final_task_obs[0] = scaled_heading;
    final_task_obs.tail(task_obs.size()) = task_obs;

    // 7. 总长度检查
    int expected_len = self_obs_len + final_task_obs.size() + task_next_obs.size() + mask_.size();
    int actual_len = observation.size();

    if (expected_len != actual_len) {
        throw std::runtime_error(
            "[MocapTask.resolveObservation] Observation dimension mismatch! "
            "Expected = " + std::to_string(expected_len) +
            ", Actual = " + std::to_string(actual_len) +
            " | self_obs = " + std::to_string(self_obs_len) +
            ", task_obs = " + std::to_string(final_task_obs.size()) +
            ", task_next_obs = " + std::to_string(task_next_obs.size()) +
            ", mask = " + std::to_string(mask_.size()));
    }

    // 8. 填充 observation
    // observation.head(self_obs_len) = observation_self_; // 来自 resolveSelfObservation()
    observation.segment(self_obs_len, final_task_obs.size()) = final_task_obs;
    observation.segment(self_obs_len + final_task_obs.size(), task_next_obs.size()) = task_next_obs;
    observation.tail(mask_.size()) = mask_;
}

MocapResult MocapTask::getMocap() {
    MocapData mocap_data = mocap_receiver_->subscribe();
    const auto& key_points_data = mocap_data.key_points;

    int num_samples = task_cfg_.num_samples;
    int num_keypoints = 30;
    int track_num = track_keypoints_indices_.size();

    Eigen::MatrixXf mocap_state = Eigen::MatrixXf::Zero(num_samples, num_keypoints * 3);

    for(int t = 0; t < num_samples; ++t){
        for (size_t j = 0; j < track_num; ++j){
            int idx = track_keypoints_indices_(j);
            const auto& kp = key_points_data[t][j];
            mocap_state(t, idx * 3 + 0) = kp[0];
            mocap_state(t, idx * 3 + 1) = kp[1];
            mocap_state(t, idx * 3 + 2) = kp[2];
        }
    }

    Eigen::VectorXf damping(num_samples -1);
    for (int i = 0; i < num_samples - 1; ++i){
        damping(i) = static_cast<float>(i + 1);
    }

    float dt = 1.0f / task_cfg_.sample_timestep_inv;
    Eigen::MatrixXf teleop_obs_eigen = tools::compute_teleop_observation(mocap_state, damping, dt);

    // 构建结果
    MocapResult result;

    // motion_state: [T][90]
    result.motion_state.resize(num_samples);
    for (int t = 0; t < num_samples; ++t) {
        result.motion_state[t].resize(num_keypoints * 3);
        for (int j = 0; j < num_keypoints * 3; ++j) {
            result.motion_state[t][j] = mocap_state(t, j);
        }
    }

    // teleop_obs: [T-1][D]
    int obs_time = teleop_obs_eigen.rows();  // T-1
    int obs_dim  = teleop_obs_eigen.cols();  // D
    result.teleop_obs.resize(obs_time);
    for (int t = 0; t < obs_time; ++t) {
        result.teleop_obs[t].resize(obs_dim);
        for (int d = 0; d < obs_dim; ++d) {
            result.teleop_obs[t][d] = teleop_obs_eigen(t, d);
        }
    }

    // 3 get visualization
    result.visualization = mocap_data.visualization;

    // sanity check: compare with subscribed teleop_obs
    // int T = result.teleop_obs.size();       // T-1
    // int D = result.teleop_obs[0].size();    // D

    // const auto& ref = mocap_data.teleop_obs;  // T rows of 90-dim

    // bool mismatch_found = false;

    // if (ref.size() != T) {
    //     FRC_ERROR("[MocapTask.getMocap] teleop_obs length mismatch: expected " << T << ", got " << ref.size());
    //     mismatch_found = true;
    // } else {
    //     for (int t = 0; t < T; ++t) {
    //         for (int d = 0; d < D; ++d) {
    //             float a = result.teleop_obs[t][d];
    //             float b = ref[t][d];
    //             if (std::abs(a - b) > 1e-5f) {
    //                 FRC_ERROR("[MocapTask.getMocap] Mismatch at t=" << t << ", d=" << d << " | a=" << a << ", b=" << b);
    //                 mismatch_found = true;
    //                 break;
    //             }
    //         }
    //         if (mismatch_found) break;
    //     }
    // }

    // if (mismatch_found) {
    //     throw std::runtime_error("[MocapTask.getMocap] Mismatched in teleop observation data.");
    // }


    return result;
}

CustomTypes::Action MocapTask::getAction(const CustomTypes::RobotData &robotData){
    CustomTypes::Action robotAction = BaseTask::getAction(robotData);
    getHandsAction(robotAction);
    return robotAction;
}

CustomTypes::Action MocapTask::getHandsAction(CustomTypes::Action& robotAction) {
    auto hands_data = mocap_receiver_->subscribeHands();

    // if not has message or hands_type not equal to dex3, do not run hands logical
    const auto& valid_flag = hands_data.at("valid");
    if (valid_flag[0] != 1.0f || hands_type_ != "dex3") {
        // robotAction.handsPosition.setOnes();
        return robotAction;  
    }

    const auto& left_wrist = hands_data.at("left_wrist_joints");
    const auto& right_wrist = hands_data.at("right_wrist_joints");
    const auto& hands_joints = hands_data.at("hands_joints");

    // 1. 写入 wrist motorPosition
    for (int i = 0; i < left_wrist_ids_.size(); ++i) {
        int idx = left_wrist_ids_[i];
        robotAction.motorPosition[idx] = left_wrist[i] / task_cfg_.action_scale;
    }
    for (int i = 0; i < right_wrist_ids_.size(); ++i) {
        int idx = right_wrist_ids_[i];
        robotAction.motorPosition[idx] = right_wrist[i] / task_cfg_.action_scale;
    }

    // 2. hands_joints 是一个长度为 14 的 VectorXf，直接赋值
    robotAction.handsPosition = hands_joints;
    robotAction.handsVelocity.setZero();
    robotAction.handsTorque.setZero();

    return robotAction;
}


void MocapTask::reset() {
    BaseTask::reset();
}

