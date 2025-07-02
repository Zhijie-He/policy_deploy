// tasks/MocapTask.cpp
#include "tasks/MocapTask.h"
#include "utility/logger.h"
#include "utility/tools.h"

MocapTask::MocapTask(std::shared_ptr<const BaseRobotConfig> cfg,
                     torch::Device device,
                     const std::string& inference_engine_type,
                     const std::string& precision)
      : BaseTask(cfg, std::make_shared<MocapTaskCfg>(), device,  inference_engine_type, precision),
        task_cfg_() 
{
    FRC_INFO("[MocapTask.Const] Created!");
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

}

void MocapTask::resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) {
    FRC_INFO("[MocapTask.resolveKeyboardInput] Key pressed: " << key);
}

void MocapTask::resolveObservation(const CustomTypes::RobotData &raw_obs) {
    updateObservation(raw_obs);
}
