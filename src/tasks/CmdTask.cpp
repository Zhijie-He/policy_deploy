// tasks/CmdTask.cpp
#include "tasks/CmdTask.h"
#include "tasks/TaskFactory.h"
#include "utility/logger.h"

CmdTask::CmdTask(float dt, torch::Device device) : BaseTask(dt, device) {
    FRC_INFO("[CmdTask.Const] Created on " << device << ", dt=" << dt);
}

void CmdTask::resolveKeyboardInput(char key) {
    FRC_INFO("[CmdTask.resolveKeyboardInput] Key pressed: " << key);
}

std::unordered_map<std::string, Eigen::MatrixXf> CmdTask::resolveObs(
    const Eigen::VectorXf& self_obs,
    const Eigen::VectorXf& raw_obs) {
    return { {"self_obs", self_obs}, {"raw_obs", raw_obs} };
}

Eigen::VectorXf CmdTask::getAction(const Eigen::VectorXf& self_obs, const Eigen::VectorXf& raw_obs) {
    resolveObs(self_obs, raw_obs);
    return Eigen::VectorXf::Zero(12); // dummy output
}

namespace {
bool registered = []() {
    TaskFactory::registerTask("CmdTask", [](float dt, torch::Device device) {
        return std::make_shared<CmdTask>(dt, device);
    });
    return true;
}();
}
