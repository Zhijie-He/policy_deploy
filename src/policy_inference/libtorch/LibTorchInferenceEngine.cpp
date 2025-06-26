// policy_inference/libtorch/LibTorchInferenceEngine.cpp
#include "policy_inference/libtorch/LibTorchInferenceEngine.h"
#include "utility/logger.h"

LibTorchInferenceEngine::LibTorchInferenceEngine(const std::string& policy_path, 
                                                 torch::Device device,
                                                 const std::string& precision)
    : BasePolicyInferenceEngine(device)
{
    try {
        module_ = torch::jit::load(policy_path);
        module_.to(device_);
        module_.eval();
    } catch (const std::exception& e) {
        FRC_ERROR("[LibTorchInferenceEngine] Failed to load model: " << e.what());
        std::exit(EXIT_FAILURE);
    }
}