// tasks/TeleopTask.cpp
#include <fstream>
#include <cmath>
#include "tasks/TeleopTask.h"
#include "utility/logger.h"
#include "utility/tools.h"


TeleopTask::TeleopTask(std::shared_ptr<const BaseRobotConfig> cfg,
                       torch::Device device,
                       const std::string& hands_type,
                       const std::string& inference_engine_type,
                       const std::string& precision)
     : BaseTask(cfg, std::make_shared<TeleopTaskCfg>(), device, hands_type, inference_engine_type, precision),
       task_cfg_() 
{
    FRC_INFO("[TeleopTask.Const] Created!");
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
        FRC_HIGHLIGHT("motion_id_" << motion_id_ << "motion_id_ + 1" << motion_id_ + 1 << "num_motions" << task_cfg_.num_motions);
    }

    FRC_INFO("[TeleopTask.resolveKeyboardInput] TeleopTask motion_id: " << motion_id_);   
}

void TeleopTask::resolveObservation(const CustomTypes::RobotData &raw_obs) {
    updateObservation(raw_obs); // get self_obs → observation.head(93)

    int self_obs_len = 93;

    // 1. cache index
    int cache_index = static_cast<int>((counter_ - count_offset_) * static_cast<float>(start_));
    cache_index = std::min(cache_index, motion_lib_cache_len_[motion_id_] - 1);

    // 2. 获取当前 teleop_obs [当前帧][动作编号]
    const auto& cache = motion_lib_cache_[cache_index]["teleop_obs"][motion_id_];

    // 3. 构造 task_obs（当前帧）
    Eigen::VectorXf task_obs(cache[0].size());
    for (size_t i = 0; i < cache[0].size(); ++i) {
        task_obs[i] = cache[0][i].get<float>();
    }

    // 4. 构造 task_next_obs（剩下 8 帧）
    Eigen::VectorXf task_next_obs((task_cfg_.num_samples - 1) * cache[0].size());
    for (int i = 1; i < task_cfg_.num_samples; ++i) {
        for (size_t j = 0; j < cache[i].size(); ++j) {
            task_next_obs[(i - 1) * cache[i].size() + j] = cache[i][j].get<float>();
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
    observation.segment(self_obs_len + final_task_obs.size(), task_next_obs.size()) = task_next_obs;
    observation.tail(mask_.size()) = mask_;
}

void TeleopTask::loadMotion() {
    std::ifstream file(task_cfg_.motion_json_file);
    if (!file.is_open()) {
        throw std::runtime_error("[TeleopTask.loadMotion] Failed to open: " + task_cfg_.motion_json_file);
    }
    json data;
    file >> data;

    // 读取 motion_lib_cache_len_
    for (const auto& val : data["motion_lib_cache_len"]) {
        motion_lib_cache_len_.push_back(val.get<int>());
    }

    // 读取 motion_lib_cache_
    motion_lib_cache_ = data["motion_lib_cache"];

    size_t total_frames = motion_lib_cache_.size(); 
    const auto& sample = motion_lib_cache_[0]["teleop_obs"];
    size_t motions = sample.size();             
    size_t frames_per_motion = sample[0].size(); 
    size_t features_per_frame = sample[0][0].size();

    FRC_INFO("[TeleopTask.loadMotion] motion_lib_cache_[teleop_obs] shape = [" << total_frames << ", " << motions << ", " << frames_per_motion << ", " << features_per_frame << "]");
}

void TeleopTask::reset() {
    BaseTask::reset();
    count_offset_ = 0;
    FRC_INFO("[TeleopTask.reset] Reset called. motion_id: " << motion_id_);
}

