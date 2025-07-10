// tasks/BaseTask.h
#pragma once
#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <torch/torch.h>
#include "types/CustomTypes.h"
#include "config/BaseRobotConfig.h" 
// #include "inference_engine/BasePolicyInferenceEngine.h"

class BasePolicyInferenceEngine;

struct BaseTaskCfg {
    float action_clip = 10.0;
    virtual ~BaseTaskCfg() = default;

    virtual std::string getPolicyPath() const = 0;
    virtual std::string getEnginePath() const = 0;
    virtual int getNumActions() const = 0;
    virtual int getNumObs() const = 0;
    virtual int getNumHidden() const = 0;
};

class BaseTask {
public:
     BaseTask(std::shared_ptr<const BaseRobotConfig> cfg,
              std::shared_ptr<const BaseTaskCfg> task_cfg, 
              torch::Device device,
              const std::string& hands_type,
              const std::string& inference_engine_type,
              const std::string& precision);
              
    virtual ~BaseTask() = default;
    CustomTypes::Action getAction(const CustomTypes::RobotData &robotData);
    
    virtual void resolveObservation(const CustomTypes::RobotData& robotData) = 0;
    virtual void resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) = 0;
    virtual std::string getVisualization(const Eigen::VectorXf&, const Eigen::VectorXf&, const Eigen::VectorXf&) { return visualization_; }
    virtual void reset();

protected:
    void updateObservation(const CustomTypes::RobotData& robotData);
    
    int obDim, acDim, handsDim;
    std::string hands_type_;
    std::shared_ptr<const BaseRobotConfig> cfg_;
    std::shared_ptr<const BaseTaskCfg> task_cfg_;
    int counter_;
    bool start_;
    
    std::string visualization_;

    Eigen::VectorXf _kP;
    Eigen::VectorXf _kD;
    Eigen::VectorXf observation;
    Eigen::VectorXf action, actionPrev;
    std::shared_ptr<BasePolicyInferenceEngine> engine_;
};

