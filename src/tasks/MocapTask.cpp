// tasks/MocapTask.cpp
#include "tasks/MocapTask.h"
#include "utility/logger.h"
#include "utility/tools.h"
#include <Eigen/Dense>

std::vector<std::vector<float>> compute_teleop_observation_test(
    const std::vector<std::vector<std::array<float, 3>>>& motion_state,
    float dt)
{
    int T = motion_state.size();          // 帧数
    int K = motion_state[0].size();       // 关键点数

    // 初始化输出：每帧 [K * 3]
    std::vector<std::vector<float>> mocap_obs(T, std::vector<float>(K * 3, 0.0f));

    // 获取第0帧 root 坐标
    std::array<float, 3> root0 = motion_state[0][0];

    for (int t = 0; t < T; ++t) {
        for (int k = 0; k < K; ++k) {
            std::array<float, 3> value = motion_state[t][k];

            // Step 1: 相对位置（减去第0帧 root）
            for (int d = 0; d < 3; ++d)
                value[d] -= root0[d];

            // Step 2: 从 t=1 开始计算速度（差分除以 dt * 5）
            if (t > 0) {
                for (int d = 0; d < 3; ++d)
                    value[d] = (motion_state[t][k][d] - motion_state[0][k][d]) / dt / 5.0f;

                // Step 3: 衰减处理 (仅 X, Y)
                float damping = static_cast<float>(t);
                value[0] /= damping;
                value[1] /= damping;
            }

            // 保存为展平形式：mocap_obs[t][k*3 + d]
            for (int d = 0; d < 3; ++d)
                mocap_obs[t][k * 3 + d] = value[d];
        }
    }

    // Step 4: 补上第0帧的 root = 第1帧的 root
    for (int d = 0; d < 3; ++d)
        mocap_obs[0][0 * 3 + d] = mocap_obs[1][0 * 3 + d];

    return mocap_obs;
}


Eigen::MatrixXf compute_teleop_observation(
    const Eigen::MatrixXf& motion_body_pos,  // [T, 90]
    const Eigen::VectorXf& damping,          // [T-1]
    float dt
) {
    int T = motion_body_pos.rows();      // 时间步数
    int D = motion_body_pos.cols();      // 90
    int J = D / 3;                       // 30

    // 输出
    Eigen::MatrixXf local_motion = Eigen::MatrixXf::Zero(T, D);

    // 1. 计算 root 起始位置（第0帧，第0个关键点的3D坐标）
    Eigen::Vector3f ref_root_start = motion_body_pos.block(0, 0, 1, 3).transpose();  // [3]

    // 2. 所有位置减去 ref_root_start，实现相对坐标系（只对 pos[t][j]）
    for (int t = 0; t < T; ++t) {
        for (int j = 0; j < J; ++j) {
            for (int d = 0; d < 3; ++d) {
                local_motion(t, j * 3 + d) = motion_body_pos(t, j * 3 + d) - ref_root_start(d);
            }
        }
    }

    // 3. 第1帧到最后一帧：计算速度并除以dt * 5
    for (int t = 1; t < T; ++t) {
        for (int j = 0; j < J; ++j) {
            for (int d = 0; d < 3; ++d) {
                float vel = (motion_body_pos(t, j * 3 + d) - motion_body_pos(0, j * 3 + d)) / dt / 5.f;
                local_motion(t, j * 3 + d) = vel;
            }
        }
    }

    // 4. 对于第1帧及以后，将 XY 平面除以 damping（不对 Z 做处理）
    for (int t = 1; t < T; ++t) {
        float damp = damping(t - 1);
        for (int j = 0; j < J; ++j) {
            local_motion(t, j * 3 + 0) /= damp;  // x
            local_motion(t, j * 3 + 1) /= damp;  // y
        }
    }

    // 5. 把第0帧的 root（第0个点）替换为第1帧的 root
    local_motion.row(0).segment(0, 3) = local_motion.row(1).segment(0, 3);

    return local_motion;  // shape: [T, 90]
}

MocapTask::MocapTask(std::shared_ptr<const BaseRobotConfig> cfg,
                     torch::Device device,
                     const std::string& inference_engine_type,
                     const std::string& precision)
      : BaseTask(cfg, std::make_shared<MocapTaskCfg>(), device,  inference_engine_type, precision),
        task_cfg_() 
{
    FRC_INFO("[MocapTask.Const] Created!");
    // overwrite cfg from model cfg
    // task_cfg_.num_samples = 1 + self.actor.cfg["task_next_obs"]["shape"][0]

    obs_scale_heading_ = task_cfg_.obs_scale_heading;
    
    // 1. 提取 track_keypoints 对应的下标索引
    const auto& body_names = task_cfg_.BODY_NAMES;
    const auto& track_names = task_cfg_.track_keypoints_names;

    for (const auto& name : track_names) {
        auto it = std::find(body_names.begin(), body_names.end(), name);
        if (it != body_names.end()) {
            track_keypoints_indices_.push_back(std::distance(body_names.begin(), it));
        } else {
            throw std::runtime_error("Body name not found in BODY_NAMES: " + name);
        }
    }
    
    // 2. 创建 mask_ 向量
    int n_bodies = static_cast<int>(body_names.size());
    mask_ = Eigen::VectorXf::Zero(n_bodies);
    for (int idx : track_keypoints_indices_) {
        mask_[idx] = 1.0f;
    }
    FRC_INFO("[MocapTask.Const] mask" << mask_.transpose());

    // 3. create mocap receiver
    mocap_receiver_ = std::make_unique<MocapMsgSubscriber>(task_cfg_.sample_timestep_inv, task_cfg_.num_samples);
}

void MocapTask::resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) {
    FRC_INFO("[MocapTask.resolveKeyboardInput] Key pressed: " << key);
}

void MocapTask::resolveObservation(const CustomTypes::RobotData &raw_obs) {
    updateObservation(raw_obs); // get self_obs → observation.head(93)

    int self_obs_len = 93;

    MocapResult mocap = getMocap();
    const auto& cache = mocap.mocap_obs;
    // const auto& points = mocap.motion_state;

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
    float scaled_heading = heading * obs_scale_heading_;

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
    // observation.head(self_obs_len) = observation_self_; // 来自 updateObservation()
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

    Eigen::MatrixXf mocap_state = Eigen::MatrixXf::Zero(num_samples, num_keypoints*3);

    for(int t = 0; t < num_samples; ++t){
        for (size_t j = 0; j < track_num; ++j){
            int idx = track_keypoints_indices_[j];
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
    Eigen::MatrixXf mocap_obs_eigen = compute_teleop_observation(mocap_state, damping, dt);

    // 5. 构建结果
    MocapResult result;

    // 5.1 motion_state: [T][90]
    result.motion_state.resize(num_samples);
    for (int t = 0; t < num_samples; ++t) {
        result.motion_state[t].resize(num_keypoints * 3);
        for (int j = 0; j < num_keypoints * 3; ++j) {
            result.motion_state[t][j] = mocap_state(t, j);
        }
    }

    // 5.2 mocap_obs: [T-1][D]
    int obs_time = mocap_obs_eigen.rows();  // T-1
    int obs_dim  = mocap_obs_eigen.cols();  // D
    result.mocap_obs.resize(obs_time);
    for (int t = 0; t < obs_time; ++t) {
        result.mocap_obs[t].resize(obs_dim);
        for (int d = 0; d < obs_dim; ++d) {
            result.mocap_obs[t][d] = mocap_obs_eigen(t, d);
        }
    }

    return result;
}

void MocapTask::reset() {
    BaseTask::reset();
}
