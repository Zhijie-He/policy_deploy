// tasks/MocapTask.h
#pragma once
#include "tasks/BaseTask.h"
#include "tasks/TaskFactory.h"

struct MocapTaskCfg : public BaseTaskCfg {
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

class MocapTask : public BaseTask {
public:
    MocapTask(std::shared_ptr<const BaseRobotConfig> cfg,
              torch::Device device,
              const std::string& inference_engine_type,
              const std::string& precision);
    void resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) override;
    void resolveObservation(const CustomTypes::RobotData& robotData) override;

private:
    MocapTaskCfg task_cfg_;
};


// Register Task
namespace {
bool registered = []() {
    TaskFactory::registerTask("MocapTask", 
        [](std::shared_ptr<const BaseRobotConfig> cfg, 
           torch::Device device,
           const std::string& inference_engine_type,
           const std::string& precision) {
            return std::make_shared<MocapTask>(cfg, device, inference_engine_type, precision);
        });
    return true;
}();
}