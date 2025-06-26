// policy_inference/PolicyInferenceEngineFactory.h
#pragma once

#include "policy_inference/BasePolicyInferenceEngine.h"
#include "policy_inference/libtorch/LibTorchInferenceEngine.h"
#include "policy_inference/tensorrt/TensorRTInferenceEngine.h"


#include <memory>
#include <string>
#include <stdexcept>

#include "utility/tools.h"

class PolicyInferenceEngineFactory {
public:
    static std::shared_ptr<BasePolicyInferenceEngine> create(
        const std::string& backend,  
        std::shared_ptr<const BaseRobotConfig> cfg,
        const torch::Device& device,
        const std::string& precision = "fp32"
    ) {
        if (backend == "libtorch") {
            return std::make_shared<LibTorchInferenceEngine>(cfg, device, precision);
        }
        else if (backend == "tensorrt") {
            return std::make_shared<TensorRTInferenceEngine>(cfg, device, precision);
        }
        else {
            FRC_ERROR("[PolicyInferenceEngineFactory.create] Unsupported inference backend type: " << backend);
            throw std::invalid_argument("Unsupported inference backend type: " + backend);
        }
    }
};
