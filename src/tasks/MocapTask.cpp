// tasks/MocapTask.cpp
#include "tasks/MocapTask.h"
#include "utility/logger.h"

MocapTask::MocapTask(float control_dt, torch::Device device)
      : BaseTask(std::make_shared<MocapTaskCfg>(), control_dt, device)
{
    FRC_INFO("[MocapTask.Const] Created on " << device << ", control_dt=" << control_dt);
}

void MocapTask::resolveKeyboardInput(char key) {
    FRC_INFO("[MocapTask.resolveKeyboardInput] Key pressed: " << key);
}

std::unordered_map<std::string, Eigen::MatrixXf> MocapTask::resolveObs(
    const Eigen::VectorXf& self_obs,
    const Eigen::VectorXf& raw_obs) {
    return { {"self_obs", self_obs}, {"raw_obs", raw_obs} };
}

