// tasks/CmdTask.h
#pragma once
#include "tasks/BaseTask.h"
#include "tasks/TaskFactory.h"

struct CmdTaskCfg : public BaseTaskCfg {
    std::string policy_path = std::string(PROJECT_SOURCE_DIR) + "/resources/policies/g1/cmdTask.pt";
    std::string engine_path = std::string(PROJECT_SOURCE_DIR) + "/resources/policies/g1/cmdTask.engine";
    
    int num_obs = 96;       // 93 + 3
    int num_actions = 29;
    int num_hidden = 2883;  // 32*93 + 3 - 96

    Eigen::Vector3f max_cmd = {0.8f, 0.5f, 1.57f};
    Eigen::Vector3f obs_scale = {2.0f, 2.0f, 0.25f};

    std::string getPolicyPath() const override{
        return policy_path;
    }

    std::string getEnginePath() const override{
        return engine_path;
    }
    
    int getNumActions() const override{
        return num_actions;
    }

    int getNumObs() const override{
        return num_obs;
    }

    int getNumHidden() const override{
        return num_hidden;
    }
};

class CmdTask : public BaseTask {
public:
    CmdTask(std::shared_ptr<const BaseRobotConfig> cfg,
            torch::Device device,
            const std::string& hands_type,
            const std::string& inference_engine_type,
            const std::string& precision);
    void resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) override;
    void resolveSelfObservation(const CustomTypes::RobotData& robotData) override;
    void resolveTaskObservation(const CustomTypes::RobotData& robotData) override;
    void reset() override;
    
private:
    CmdTaskCfg task_cfg_;
    std::mutex cmd_states_lock_;
    Eigen::Vector3f cmd_states_;          // 初始为 0
    Eigen::Vector3f max_cmd_;             // 从 cfg_ 读取
    Eigen::Vector3f cmd_obs_scale_;       // 从 cfg_ 读取
};

// Register Task
namespace {
bool registered = []() {
    TaskFactory::registerTask("CmdTask", 
        [](std::shared_ptr<const BaseRobotConfig> cfg, 
           torch::Device device,
           const std::string& hands_type,
           const std::string& inference_engine_type,
           const std::string& precision) {
            return std::make_shared<CmdTask>(cfg, device, hands_type, inference_engine_type, precision);
        });
    return true;
}();
}