#pragma once
#include "config/BaseRobotConfig.h" 
#include "policy_inference/BasePolicyInferenceEngine.h"
#include "types/CustomTypes.h"
#include "types/cpp_types.h"
#include <torch/script.h>

class BasePolicyWrapper
{
public:
    BasePolicyWrapper(std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device);
    virtual CustomTypes::Action getControlAction(const CustomTypes::RobotData &robotData) = 0;
    virtual ~BasePolicyWrapper() = default;

protected:
    int obDim, acDim;
    torch::Device device_ = torch::kCPU;  // 全局 device 成员
    
    std::shared_ptr<BasePolicyInferenceEngine> engine_;

    torch::jit::script::Module module_;
    torch::Tensor obTorch, acTorch;
    std::vector<c10::IValue> obVector{}; 
    std::shared_ptr<const BaseRobotConfig> cfg_; 
    
    Eigen::VectorXf _kP;
    Eigen::VectorXf _kD;

    Eigen::VectorXf observation;
    Eigen::VectorXf action, actionPrev;
};