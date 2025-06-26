#pragma once
#include "config/BaseRobotConfig.h" 
#include "policy_inference/BasePolicyInferenceEngine.h"
#include "types/CustomTypes.h"
#include "types/cpp_types.h"
#include <torch/script.h>

class BasePolicyWrapper
{
public:
    BasePolicyWrapper(std::shared_ptr<const BaseRobotConfig> cfg, 
                      torch::Device device, 
                      const std::string& inference_engine_type,
                      const std::string& precision);
    virtual CustomTypes::Action getControlAction(const CustomTypes::RobotData &robotData) = 0;
    virtual ~BasePolicyWrapper() = default;

protected:
    int obDim, acDim;
    
    std::shared_ptr<BasePolicyInferenceEngine> engine_;
    std::shared_ptr<const BaseRobotConfig> cfg_; 
    
    Eigen::VectorXf _kP;
    Eigen::VectorXf _kD;

    Eigen::VectorXf observation;
    Eigen::VectorXf action, actionPrev;
};