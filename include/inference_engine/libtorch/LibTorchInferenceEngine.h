// inference_enginelibtorch/LibTorchInferenceEngine.h
#pragma once

#include "inference_engine/BasePolicyInferenceEngine.h"
#include <memory>
#include <iostream>

class LibTorchInferenceEngine : public BasePolicyInferenceEngine {
public:
    LibTorchInferenceEngine(std::shared_ptr<const BaseRobotConfig> cfg,
                            torch::Device device,
                            const std::string& precision);
    void loadModel();
    void warmUp(int rounds = 10) override;
    void reset(const std::string& method_name = "") override;
    Eigen::VectorXf predict(const Eigen::VectorXf& observation) override;

private:
    torch::Device device_;
    torch::Dtype precision_ = torch::kFloat32; // default precision
    torch::jit::Module module_;
    torch::Tensor obTorch, acTorch;
    std::vector<c10::IValue> obVector{}; 

    Eigen::VectorXf prev_hidden_state_;  
    int hiddenDim = 0; 
};
