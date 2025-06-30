// tasks/TeleopTask.cpp
#include "tasks/TeleopTask.h"
#include "tasks/TaskFactory.h"
#include "utility/logger.h"

TeleopTask::TeleopTask(float dt, torch::Device device) : BaseTask(dt, device) {
    FRC_INFO("[TeleopTask.Const] Created on " << device << ", dt=" << dt);
}

void TeleopTask::resolveKeyboardInput(char key) {
    FRC_INFO("[TeleopTask.resolveKeyboardInput] Key pressed: " << key);
}

std::unordered_map<std::string, Eigen::MatrixXf> TeleopTask::resolveObs(
    const Eigen::VectorXf& self_obs,
    const Eigen::VectorXf& raw_obs) {
    return { {"self_obs", self_obs}, {"raw_obs", raw_obs} };
}

Eigen::VectorXf TeleopTask::getAction(const Eigen::VectorXf& self_obs, const Eigen::VectorXf& raw_obs) {
    resolveObs(self_obs, raw_obs);
    return Eigen::VectorXf::Ones(12); // dummy action output
}

namespace {
bool registered = []() {
    TaskFactory::registerTask("TeleopTask", [](float dt, torch::Device device) {
        return std::make_shared<TeleopTask>(dt, device);
    });
    return true;
}();
}

