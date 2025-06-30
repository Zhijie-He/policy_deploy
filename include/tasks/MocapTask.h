// tasks/MocapTask.h
#pragma once
#include "tasks/BaseTask.h"

class MocapTask : public BaseTask {
public:
    MocapTask(float dt, torch::Device device);

    void resolveKeyboardInput(char key) override;
    std::unordered_map<std::string, Eigen::MatrixXf> resolveObs(
        const Eigen::VectorXf& self_obs,
        const Eigen::VectorXf& raw_obs) override;
    Eigen::VectorXf getAction(const Eigen::VectorXf& self_obs, const Eigen::VectorXf& raw_obs) override;
};

