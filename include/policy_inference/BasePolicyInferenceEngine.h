// policy_inference/BasePolicyInferenceEngine.h
#pragma once

#include <torch/script.h>
#include <Eigen/Core>
#include <string>
#include <stdexcept>

// 推理引擎的通用接口
class BasePolicyInferenceEngine {
public:
    BasePolicyInferenceEngine(torch::Device device);
    virtual ~BasePolicyInferenceEngine() = default;
    torch::jit::Module module_;
    
protected:
    torch::Device device_;
};