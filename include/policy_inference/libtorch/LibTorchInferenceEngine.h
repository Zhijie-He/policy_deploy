// policy_inference/libtorch/LibTorchInferenceEngine.h
#pragma once

#include "policy_inference/BasePolicyInferenceEngine.h"
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
};
