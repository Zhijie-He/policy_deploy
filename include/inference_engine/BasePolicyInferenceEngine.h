// inference_engine/BasePolicyInferenceEngine.h
#pragma once

#include <torch/script.h>
#include <Eigen/Core>
#include <string>
#include <stdexcept>
#include "config/BaseRobotConfig.h" 
// #include "tasks/BaseTask.h" 

struct BaseTaskCfg;

// 推理引擎的通用接口
class BasePolicyInferenceEngine {
public:
    BasePolicyInferenceEngine(std::shared_ptr<const BaseRobotConfig> cfg, 
                              std::shared_ptr<const BaseTaskCfg> task_cfg, 
                              torch::Device device, 
                              const std::string& precision);

    virtual ~BasePolicyInferenceEngine() = default;
    virtual void warmUp(int rounds = 10) {};
    virtual void reset(const std::string& method_name = "");
    virtual Eigen::VectorXf predict(const Eigen::VectorXf& observation) = 0;
    
protected:
    int obDim, acDim;
    int hiddenDim = 0; 
    Eigen::VectorXf prev_hidden_state_;
    
    std::string precision_str_;
    std::shared_ptr<const BaseRobotConfig> cfg_; 
    std::shared_ptr<const BaseTaskCfg> task_cfg_;
    std::string policy_path_;
    std::string engine_path_;
};