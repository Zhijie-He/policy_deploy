// tasks/TeleopTask.h
#pragma once
#include "tasks/BaseTask.h"
#include "tasks/TaskFactory.h"

struct TeleopTaskCfg : public BaseTaskCfg {
    std::string actor = std::string(PROJECT_SOURCE_DIR) + "/resources/g1/actor.pt";

    struct Obs {
        int self_obs = 93;
        int task_obs = 3;
    } obs;

    Eigen::Vector3f max_cmd = {0.8f, 0.5f, 1.57f};
    Eigen::Vector3f obs_scale = {2.0f, 2.0f, 0.25f};
};


class TeleopTask : public BaseTask {
public:
    TeleopTask(float control_dt, const torch::Device device);

    void resolveKeyboardInput(char key) override;
    std::unordered_map<std::string, Eigen::MatrixXf> resolveObs(
        const Eigen::VectorXf& self_obs,
        const Eigen::VectorXf& raw_obs) override;
};

namespace {
bool registered = []() {
    TaskFactory::registerTask("TeleopTask", [](float control_dt, torch::Device device) {
        return std::make_shared<TeleopTask>(control_dt, device);
    });
    return true;
}();
}

