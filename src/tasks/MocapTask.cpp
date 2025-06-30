// tasks/MocapTask.cpp
#include "tasks/MocapTask.h"
#include "tasks/TaskFactory.h"
#include "utility/logger.h"

MocapTask::MocapTask(float dt, torch::Device device) : BaseTask(dt, device) {
    FRC_INFO("[MocapTask.Const] Created on " << device << ", dt=" << dt);
}

void MocapTask::resolveKeyboardInput(char key) {
    FRC_INFO("[MocapTask.resolveKeyboardInput] Key pressed: " << key);
}

std::unordered_map<std::string, Eigen::MatrixXf> MocapTask::resolveObs(
    const Eigen::VectorXf& self_obs,
    const Eigen::VectorXf& raw_obs) {
    return { {"self_obs", self_obs}, {"raw_obs", raw_obs} };
}

Eigen::VectorXf MocapTask::getAction(const Eigen::VectorXf& self_obs, const Eigen::VectorXf& raw_obs) {
    resolveObs(self_obs, raw_obs);
    return Eigen::VectorXf::Ones(12); // dummy action output
}

namespace {
bool registered = []() {
    TaskFactory::registerTask("MocapTask", [](float dt, torch::Device device) {
        return std::make_shared<MocapTask>(dt, device);
    });
    return true;
}();
}

