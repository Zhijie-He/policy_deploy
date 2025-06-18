#pragma once
#include "config/BaseRobotConfig.h" 
#include "types/CustomTypes.h"
#include "types/cpp_types.h"
#include <torch/script.h>

class NeuralController
{
public:
    NeuralController(std::shared_ptr<const BaseRobotConfig> cfg);
    virtual CustomTypes::Action getControlAction(const CustomTypes::RobotData &robotData) = 0;
    virtual ~NeuralController() = default;

protected:
    int obDim, acDim;
    torch::Device device_ = torch::kCPU;  // 全局 device 成员
    torch::jit::script::Module module_;
    torch::Tensor obTorch, acTorch;
    std::vector<c10::IValue> obVector{}; 
    std::shared_ptr<const BaseRobotConfig> cfg_; 
    
    Eigen::VectorXf _kP;
    Eigen::VectorXf _kD;

    Eigen::VectorXf observation;
    Eigen::VectorXf action, actionPrev;
    
    int infer_count=0;
    double infer_sum_us=0;
    double infer_sum_sq_us=0;
};