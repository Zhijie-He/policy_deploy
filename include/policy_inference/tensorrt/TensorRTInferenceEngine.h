// policy_inference/tensorrt/TensorRTInferenceEngine.h
#pragma once

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <cassert>
#include <numeric>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <cuda_fp16.h>
#include "policy_inference/BasePolicyInferenceEngine.h"
#include "utility/logger.h"

using namespace nvinfer1;

// ---------- Smart Destroy Helper ----------
template <typename T>
struct TRTDestroyer {
    void operator()(T* obj) const {
        if (obj) obj->destroy();
    }
};

// ---------- Logger ----------
class Logger : public ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            FRC_INFO("[TensorRTInferenceEngine.log] " << msg);
    }
};

class TensorRTInferenceEngine : public BasePolicyInferenceEngine {
public:
    explicit TensorRTInferenceEngine(std::shared_ptr<const BaseRobotConfig> cfg,
                            torch::Device device,
                            const std::string& precision);
    void loadModel();
    void warmUp(int rounds = 10) override;
    void reset(const std::string& method_name = "") override;
    Eigen::VectorXf predict(const Eigen::VectorXf& observation) override;

private:
    Logger logger_;
    cudaStream_t stream_{nullptr};

    std::unique_ptr<IRuntime, TRTDestroyer<IRuntime>> runtime_;
    std::unique_ptr<ICudaEngine, TRTDestroyer<ICudaEngine>> engine_;
    std::unique_ptr<IExecutionContext, TRTDestroyer<IExecutionContext>> context_;
    
    DataType input_dtype_;
    DataType output_dtype_;
    
    void* buffers_[2]{nullptr, nullptr};
    int inputIndex_{0};
    int outputIndex_{1};
    size_t inputSize_{0};
    size_t outputSize_{0};

    int volume(const Dims& dims) const {
        return std::accumulate(dims.d, dims.d + dims.nbDims, 1, std::multiplies<int>());
    }

    std::vector<char> loadEngineFile(const std::string& path)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file) throw std::runtime_error("Failed to open engine file: " + path);
        return std::vector<char>(std::istreambuf_iterator<char>(file), {});
    }

    Eigen::VectorXf prev_hidden_state_;
};
