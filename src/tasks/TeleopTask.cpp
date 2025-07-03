// tasks/TeleopTask.cpp
#include <fstream>
#include <cmath>
#include "tasks/TeleopTask.h"
#include "utility/logger.h"
#include "utility/tools.h"
#include "utility/json.hpp"

using json = nlohmann::json;

TeleopTask::TeleopTask(std::shared_ptr<const BaseRobotConfig> cfg,
                       torch::Device device,
                       const std::string& inference_engine_type,
                       const std::string& precision)
     : BaseTask(cfg, std::make_shared<TeleopTaskCfg>(), device,  inference_engine_type, precision),
       task_cfg_() 
{
    FRC_INFO("[TeleopTask.Const] Created!");
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
    FRC_INFO("[TeleopTask.Const] mask" << mask_.transpose());

    motion_id_ = 0;
    count_offset_ = 0;

    // 3. 加载运动序列
    loadMotion();
}

void TeleopTask::resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) {
    if (key == 'b'){
        start_ = true;
        count_offset_ = counter_;
    } else if (key == 'n'){
        start_ = false;
        count_offset_ = counter_;
        motion_id_ = (motion_id_ + 1) % task_cfg_.num_motions; 
    }

    FRC_INFO("[TeleopTask.resolveKeyboardInput] TeleopTask motion_id: " << motion_id_);   
}

void TeleopTask::resolveObservation(const CustomTypes::RobotData &raw_obs) {
    updateObservation(raw_obs); // get self_obs → observation.head(93)

    int self_obs_len = 93;

    // 1. cache index
    int cache_index = static_cast<int>((counter_ - count_offset_) * static_cast<float>(start_));
    cache_index = std::min(cache_index, motion_lib_cache_len_[motion_id_] - 1);
    // 2. 获取当前的 motion 数据块
    const auto& cache = motion_lib_cache_[cache_index];
    const auto& motion_obs = cache.teleop_obs[motion_id_];  // [timesteps][body_points]

    // 3. 构造 task_obs
    const auto& first_frame = motion_obs[0];  // std::vector<Eigen::Vector3f>
    Eigen::VectorXf task_obs(first_frame.size() * 3);
    for (size_t i = 0; i < first_frame.size(); ++i) {
        task_obs.segment<3>(i * 3) = first_frame[i];
    }

    // 4. 构造 task_next_obs
    int num_next = static_cast<int>(task_cfg_.num_samples - 1);
    int obs_dim = static_cast<int>(first_frame.size() * 3);
    Eigen::MatrixXf task_next_obs(num_next, obs_dim);
    for (int t = 0; t < num_next; ++t) {
        const auto& frame = motion_obs[t + 1];
        for (size_t i = 0; i < frame.size(); ++i) {
            task_next_obs.block(t, i * 3, 1, 3) = frame[i].transpose();
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
            "[TeleopTask.resolveObservation] Observation dimension mismatch! "
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
    observation.segment(self_obs_len + final_task_obs.size(), task_next_obs.size()) = Eigen::Map<const Eigen::VectorXf>(task_next_obs.data(), task_next_obs.size());
    observation.tail(mask_.size()) = mask_;
}

void TeleopTask::loadMotion() {
    std::ifstream file(task_cfg_.motion_json_file);
    if (!file.is_open()) {
        throw std::runtime_error("[TeleopTask.loadMotion] Failed to open: " + task_cfg_.motion_json_file);
    }

    json j;
    file >> j;

    // 1. 加载 motion_lib_cache_len_
    motion_lib_cache_len_ = j["motion_lib_cache_len"].get<std::vector<int>>();

    // 2. 加载 motion_lib_cache_
    motion_lib_cache_.clear();
    for (const auto& cache_entry : j["motion_lib_cache"]) {
        MotionCache cache;

        // 2.1 teleop_obs: [motion][timestep][body_point(xyz)]
        const auto& obs_array = cache_entry["teleop_obs"];
        std::vector<std::vector<std::vector<Eigen::Vector3f>>> teleop_obs_all;

        for (const auto& motion_data : obs_array) {
            std::vector<std::vector<Eigen::Vector3f>> motion_sequence;
            for (const auto& timestep_data : motion_data) {
                std::vector<Eigen::Vector3f> frame;
                for (const auto& point_xyz : timestep_data) {
                    if (point_xyz.size() != 3) {
                        throw std::runtime_error("[loadMotion] teleop_obs: point must be length 3");
                    }
                    Eigen::Vector3f vec;
                    for (int i = 0; i < 3; ++i) {
                        vec[i] = static_cast<float>(point_xyz[i]);
                    }
                    frame.push_back(vec);
                }
                motion_sequence.push_back(frame);
            }
            teleop_obs_all.push_back(motion_sequence);
        }
        cache.teleop_obs = std::move(teleop_obs_all);

        // 2.2 motion_state > body_pos: [motion][timestep][features]
        // const auto& pos_array = cache_entry["motion_state"]["body_pos"];
        // std::vector<std::vector<Eigen::VectorXf>> body_positions;

        // for (const auto& motion_list : pos_array) {
        //     std::vector<Eigen::VectorXf> points;
        //     for (const auto& point : motion_list) {
        //         Eigen::VectorXf vec(point.size());
        //         for (int i = 0; i < point.size(); ++i) {
        //             vec[i] = static_cast<float>(point[i]);
        //         }
        //         points.push_back(vec);
        //     }
        //     body_positions.push_back(points);
        // }
        // cache.body_pos = std::move(body_positions);

        motion_lib_cache_.push_back(std::move(cache));
    }

    FRC_INFO("[TeleopTask.loadMotion] Loaded " << motion_lib_cache_.size() << " entries.");
}

void TeleopTask::reset() {
    BaseTask::reset();
    count_offset_ = 0;
    FRC_INFO("[TeleopTask.reset] Reset called. motion_id: " << motion_id_);
}
