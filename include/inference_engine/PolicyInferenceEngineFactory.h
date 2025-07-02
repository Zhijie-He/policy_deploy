// inference_enginePolicyInferenceEngineFactory.h
#pragma once

#include "inference_engine/BasePolicyInferenceEngine.h"
#include "inference_engine/libtorch/LibTorchInferenceEngine.h"
#ifdef USE_TENSORRT
#include "inference_engine/tensorrt/TensorRTInferenceEngine.h"
#endif
#include <memory>
#include <string>
#include <stdexcept>
#include "utility/tools.h"

class PolicyInferenceEngineFactory {
public:
    static std::shared_ptr<BasePolicyInferenceEngine> create(
        const std::string& backend,  
        std::shared_ptr<const BaseRobotConfig> cfg,
        std::shared_ptr<const BaseTaskCfg> task_cfg, 
        const torch::Device& device,
        const std::string& precision = "fp32"
    ) {
        if (backend == "libtorch") {
            return std::make_shared<LibTorchInferenceEngine>(cfg, task_cfg, device, precision);
        }
#ifdef USE_TENSORRT
        else if (backend == "tensorrt") {
            return std::make_shared<TensorRTInferenceEngine>(cfg, task_cfg, device, precision);
        }
#endif
        else {
            FRC_ERROR("[PolicyInferenceEngineFactory.create] Unsupported inference backend type: " << backend);
            throw std::invalid_argument("Unsupported inference backend type: " + backend);
        }
    }
};
