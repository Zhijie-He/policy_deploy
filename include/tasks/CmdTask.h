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
    CmdTask(std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device);
    void resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) override;
    void resolveObservation(const CustomTypes::RobotData& robotData) override;
    void reset() override;
    
private:
    CmdTaskCfg task_cfg_;
    std::mutex cmd_states_mutex_;
    Eigen::Vector3f cmd_states_;          // 初始为 0
    Eigen::Vector3f max_cmd_;             // 从 cfg_ 读取
    Eigen::Vector3f cmd_obs_scale_;       // 从 cfg_ 读取
    float yawTarg = 0;
};

// Register Task
namespace {
bool registered = []() {
    TaskFactory::registerTask("CmdTask", [](std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device) {
        return std::make_shared<CmdTask>(cfg, device);
    });
    return true;
}();
}