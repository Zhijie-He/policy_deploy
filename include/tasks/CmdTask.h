// tasks/CmdTask.h
#pragma once
#include "tasks/BaseTask.h"
#include "tasks/TaskFactory.h"

struct CmdTaskCfg : public BaseTaskCfg {
    std::string actor = std::string(PROJECT_SOURCE_DIR) + "/resources/g1/actor.pt";

    struct Obs {
        int self_obs = 93;
        int task_obs = 3;
    } obs;

    Eigen::Vector3f max_cmd = {0.8f, 0.5f, 1.57f};
    Eigen::Vector3f obs_scale = {2.0f, 2.0f, 0.25f};
};

class CmdTask : public BaseTask {
public:
    CmdTask(float control_dt, torch::Device device);

    void resolveKeyboardInput(char key) override;
    std::unordered_map<std::string, Eigen::MatrixXf> resolveObs(
        const Eigen::VectorXf& self_obs,
        const Eigen::VectorXf& raw_obs) override;
    void reset() override;
    
private:
    CmdTaskCfg cfg_;
    std::mutex cmd_states_mutex_;
    Eigen::Vector3f cmd_states_;          // 初始为 0
    Eigen::Vector3f max_cmd_;             // 从 cfg_ 读取
    Eigen::Vector3f cmd_obs_scale_;       // 从 cfg_ 读取

};

// 注册
namespace {
bool registered = []() {
    TaskFactory::registerTask("CmdTask", [](float control_dt, torch::Device device) {
        return std::make_shared<CmdTask>(control_dt, device);
    });
    return true;
}();
}