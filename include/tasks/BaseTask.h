// tasks/BaseTask.h
#pragma once
#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <torch/torch.h>
#include "types/CustomTypes.h"
#include "config/BaseRobotConfig.h" 
#include "inference_engine/BasePolicyInferenceEngine.h"

struct BaseTaskCfg {
    virtual ~BaseTaskCfg() = default;
};

class BaseTask {
public:
     BaseTask(std::shared_ptr<const BaseRobotConfig> cfg,
              std::shared_ptr<BaseTaskCfg> task_cfg, 
              torch::Device device);
    virtual ~BaseTask() = default;
    CustomTypes::Action getAction(const CustomTypes::RobotData &robotData);
    
    virtual void resolveObservation(const CustomTypes::RobotData& robotData) = 0;
    virtual void resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) = 0;
    virtual std::string getVisualization(const Eigen::VectorXf&, const Eigen::VectorXf&, const Eigen::VectorXf&) { return visualization_; }
    virtual void reset() { counter_ = 0; start_ = false; }

protected:
    int obDim, acDim;
    std::shared_ptr<const BaseRobotConfig> cfg_;
    std::shared_ptr<BaseTaskCfg> task_cfg_;
    float control_dt_;
    torch::Device device_;
    int counter_;
    bool start_;
    std::string visualization_;


    Eigen::VectorXf _kP;
    Eigen::VectorXf _kD;

    Eigen::VectorXf observation;
    Eigen::VectorXf action, actionPrev;

    std::shared_ptr<BasePolicyInferenceEngine> engine_;
};

