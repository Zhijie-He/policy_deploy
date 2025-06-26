// policy_inference/libtorch/LibTorchInferenceEngine.h
#pragma once

#include "policy_inference/BasePolicyInferenceEngine.h"
#include <memory>
#include <iostream>

class LibTorchInferenceEngine : public BasePolicyInferenceEngine {
public:
    LibTorchInferenceEngine(const std::string& policy_path, 
                            torch::Device device,
                            const std::string& precision);
};
