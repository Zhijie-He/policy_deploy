// tasks/CmdTask.cpp
#include "tasks/CmdTask.h"
#include "utility/logger.h"
#include "utility/tools.h"
#include "types/cpp_types.h"

CmdTask::CmdTask(std::shared_ptr<const BaseRobotConfig> cfg,
                 torch::Device device,
                 const std::string& hands_type,
                 const std::string& inference_engine_type,
                 const std::string& precision)
    : BaseTask(cfg, std::make_shared<CmdTaskCfg>(), device, hands_type, inference_engine_type, precision), 
      task_cfg_() 
{
    FRC_INFO("[CmdTask.Const] Created!");
    // if (!cfg_->cmd_init.isZero()) {
    //     cmd_states_ = cfg_->cmd_init;
    //     FRC_INFO("[CmdTask.Const] Initial target cmd: " << cmd_states_.transpose()); 
    // } else {
    //     cmd_states_.setZero();  
    // }

    cmd_states_.setZero();  
    max_cmd_ = task_cfg_.max_cmd;
    cmd_obs_scale_ = task_cfg_.obs_scale;
}

void CmdTask::resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) {
    constexpr float kStep = 0.05f;
    constexpr float kYawStep = 0.05f;
    constexpr float kThresh = 1e-2f;

    Vec3f deltaVel{0, 0, 0};
    float deltaYaw = 0.f;

    if (key == 'w') deltaVel << kStep, 0, 0;
    if (key == 's') deltaVel << -kStep, 0, 0;
    if (key == 'a') deltaVel << 0, kStep, 0;
    if (key == 'd') deltaVel << 0, -kStep, 0;
    if (key == 'q') deltaYaw = kYawStep;
    if (key == 'e') deltaYaw = -kYawStep;

    std::lock_guard<std::mutex> lock(cmd_states_lock_);

    // 更新线速度
    if (deltaVel.norm() > kThresh) {
        cmd_states_.head<2>() += deltaVel.head<2>();
        cmd_states_.head<2>() = cmd_states_.head<2>().cwiseMax(-max_cmd_.head<2>()).cwiseMin(max_cmd_.head<2>());
    }

    // 更新 yaw
    if (std::abs(deltaYaw) > kThresh) {
        float yaw = cmd_states_[2] + deltaYaw;
        yaw = std::clamp(yaw, -1.0f, 1.0f);  // 限制 yaw 范围
        if (std::abs(yaw) < kThresh) yaw = 0.f;
        cmd_states_[2] = yaw;
    }

    if (key == ' ') {
        cmd_states_.setZero();  // 空格重置
    }

    FRC_INFO("[CmdTask.resolveKeyboardInput] Cmd state = " << cmd_states_.transpose());
}

void CmdTask::resolveObservation(const CustomTypes::RobotData& raw_obs) {
    updateObservation(raw_obs);

    Vec3f task_obs;
    {
        std::lock_guard<std::mutex> lock(cmd_states_lock_);
        task_obs = cmd_states_.cwiseProduct(cmd_obs_scale_);
    }
    
    observation.segment(6 + 3 * acDim, 3)      = task_obs;
}

void CmdTask::reset() {
    BaseTask::reset();
    std::lock_guard<std::mutex> lock(cmd_states_lock_);
    cmd_states_.setZero();
    FRC_INFO("[CmdTask.reset] Reset called. cmd_states_ set zero");
}

