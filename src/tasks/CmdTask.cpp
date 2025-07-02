// tasks/CmdTask.cpp
#include "tasks/CmdTask.h"
#include "utility/logger.h"
#include "utility/tools.h"
#include "types/cpp_types.h"

CmdTask::CmdTask(std::shared_ptr<const BaseRobotConfig> cfg,
                 torch::Device device,
                 const std::string& inference_engine_type,
                 const std::string& precision)
    : BaseTask(cfg, std::make_shared<CmdTaskCfg>(), device, inference_engine_type, precision), 
      task_cfg_()  // 默认构造即可
{
    FRC_INFO("[CmdTask.Const] Created on " << device << ", control_dt=" << control_dt_);
    if (!cfg_->cmd_init.isZero()) {
        cmd_states_ = cfg_->cmd_init;
        FRC_INFO("[CmdTask.Const] Initial target cmd: " << cmd_states_.transpose()); 
    } else {
        cmd_states_.setZero();  
    }
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

    std::lock_guard<std::mutex> lock(cmd_states_mutex_);

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
    Eigen::Vector3f projected_gravity_b = tools::quat_rotate_inverse_on_gravity(raw_obs.root_rot * cfg_->obs_scale_projected_gravity_b);
    Eigen::Vector3f root_ang_vel_b = raw_obs.root_ang_vel * cfg_->ang_vel_scale;
    Eigen::VectorXf joint_pos = (raw_obs.joint_pos - cfg_->default_angles) * cfg_->dof_pos_scale;
    Eigen::VectorXf joint_vel = raw_obs.joint_vel * cfg_->dof_vel_scale;
    Eigen::VectorXf last_action = actionPrev * cfg_->action_scale;

    Vec3f scaled_cmd;
    {
        std::lock_guard<std::mutex> lock(cmd_states_mutex_);
        scaled_cmd = cmd_states_.cwiseProduct(cfg_->cmd_scale);
    }

    observation.segment(0, 3)                  = projected_gravity_b;
    observation.segment(3, 3)                  = root_ang_vel_b;
    observation.segment(6, acDim)              = joint_pos(cfg_->env2actor);
    observation.segment(6 + acDim, acDim)      = joint_vel(cfg_->env2actor);
    observation.segment(6 + 2 * acDim, acDim)  = actionPrev;
    observation.segment(6 + 3 * acDim, 3)      = scaled_cmd;
}

void CmdTask::reset() {
    BaseTask::reset();
    std::lock_guard<std::mutex> lock(cmd_states_mutex_);
    cmd_states_.setZero();
}

