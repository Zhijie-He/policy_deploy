// tasks/TeleopTask.cpp
#include "tasks/TeleopTask.h"
#include "utility/logger.h"

TeleopTask::TeleopTask(float control_dt, torch::Device device) 
     : BaseTask(std::make_shared<TeleopTaskCfg>(), control_dt, device)
{
    FRC_INFO("[TeleopTask.Const] Created on " << device << ", control_dt=" << control_dt);
}

void TeleopTask::resolveKeyboardInput(char key) {
    FRC_INFO("[TeleopTask.resolveKeyboardInput] Key pressed: " << key);
}

std::unordered_map<std::string, Eigen::MatrixXf> TeleopTask::resolveObs(
    const Eigen::VectorXf& self_obs,
    const Eigen::VectorXf& raw_obs) {
    return { {"self_obs", self_obs}, {"raw_obs", raw_obs} };
}
