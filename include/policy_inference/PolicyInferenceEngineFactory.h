// policy_inference/PolicyInferenceEngineFactory.h
#pragma once

#include "policy_inference/BasePolicyInferenceEngine.h"
#include "policy_inference/libtorch/LibTorchInferenceEngine.h"
#ifdef USE_TENSORRT
#include "policy_inference/tensorrt/TensorRTInferenceEngine.h"
#endif

#include <memory>
#include <string>
#include <stdexcept>

class PolicyInferenceEngineFactory {
public:
    static std::shared_ptr<BasePolicyInferenceEngine> create(
        const std::string& backend,  
        const std::string& policy_path,
        const torch::Device& device,
        const std::string& precision = "fp32"
    ) {
        if (backend == "libtorch") {
            return std::make_shared<LibTorchInferenceEngine>(policy_path, device, precision);
        }
#ifdef USE_TENSORRT
        else if (backend == "tensorrt") {
            return std::make_shared<TensorRTInferenceEngine>(policy_path, device, precision);
        }
#endif
        else {
            throw std::invalid_argument("Unsupported inference backend type: " + backend);
        }
    }
};
