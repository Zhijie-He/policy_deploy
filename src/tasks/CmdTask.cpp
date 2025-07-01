// tasks/CmdTask.cpp
#include "tasks/CmdTask.h"
#include "utility/logger.h"

CmdTask::CmdTask(float control_dt, torch::Device device)
    : BaseTask(std::make_shared<CmdTaskCfg>(), control_dt, device),  // 初始化父类里的 cfg_
      cfg_()  // 默认构造即可
{
    FRC_INFO("[CmdTask.Const] Created on " << device << ", control_dt=" << control_dt);
    cmd_states_.setZero();  // 全部初始化为 0
    max_cmd_ = cfg_.max_cmd;
    cmd_obs_scale_ = cfg_.obs_scale;
}


void CmdTask::resolveKeyboardInput(char key) {
    std::lock_guard<std::mutex> lock(cmd_states_mutex_);
    switch (key) {
        case ' ': cmd_states_.setZero(); break;
        case 'w': cmd_states_[0] += 0.05f; break;
        case 's': cmd_states_[0] -= 0.05f; break;
        case 'a': cmd_states_[1] += 0.05f; break;
        case 'd': cmd_states_[1] -= 0.05f; break;
        case ',': cmd_states_[2] += 0.05f; break;
        case '.': cmd_states_[2] -= 0.05f; break;
        default:  cmd_states_.setZero(); break;
    }

    // clip and round
    for (int i = 0; i < 3; ++i) {
        float val = std::round(cmd_states_[i] * 100.0f) / 100.0f;
        cmd_states_[i] = std::clamp(val, -max_cmd_[i], max_cmd_[i]);
    }

    FRC_INFO("[CmdTask] cmd_states: " << cmd_states_.transpose());
}

std::unordered_map<std::string, Eigen::MatrixXf> CmdTask::resolveObs(
    const Eigen::VectorXf& self_obs,
    const Eigen::VectorXf& raw_obs)
{
    Eigen::Vector3f cmd;
    {
        std::lock_guard<std::mutex> lock(cmd_states_mutex_);
        cmd = cmd_states_;
    }

    Eigen::Vector3f task_obs = cmd.cwiseProduct(cmd_obs_scale_);

    Eigen::MatrixXf task_obs_mat(1, 3);
    task_obs_mat << task_obs.transpose();  // shape: [1 x 3]

    Eigen::MatrixXf self_obs_mat(1, self_obs.size());
    self_obs_mat = self_obs.transpose();  // shape: [1 x N]

    return {
        {"self_obs", self_obs_mat},
        {"task_obs", task_obs_mat}
    };
}

void CmdTask::reset() {
    BaseTask::reset();  // 调用父类的重置逻辑

    std::lock_guard<std::mutex> lock(cmd_states_mutex_);
    cmd_states_.setZero();  // cmd 状态归零
}