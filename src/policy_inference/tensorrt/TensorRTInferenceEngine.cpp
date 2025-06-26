// policy_inference/tensorrt/TensorRTInferenceEngine.cpp
#include "policy_inference/tensorrt/TensorRTInferenceEngine.h"

TensorRTInferenceEngine::TensorRTInferenceEngine(std::shared_ptr<const BaseRobotConfig> cfg,
                                                 torch::Device device,
                                                 const std::string& precision)
    : BasePolicyInferenceEngine(cfg, device, precision)
{
    loadModel();
}

void TensorRTInferenceEngine::loadModel(){
    std::vector<char> engineData = loadEngineFile(cfg_->engine_path);
    runtime_ = std::unique_ptr<IRuntime, TRTDestroyer<IRuntime>>(createInferRuntime(logger_));
    engine_ = std::unique_ptr<ICudaEngine, TRTDestroyer<ICudaEngine>>(runtime_->deserializeCudaEngine(engineData.data(), engineData.size()));
    context_ = std::unique_ptr<IExecutionContext, TRTDestroyer<IExecutionContext>>(engine_->createExecutionContext());

    cudaStreamCreate(&stream_);

    int nbBindings = engine_->getNbBindings();
    for (int i = 0; i < nbBindings; ++i) {
        const char* name = engine_->getBindingName(i);
        if (engine_->bindingIsInput(i)) {
            inputIndex_ = i;
            FRC_INFO("[TensorRTInferenceEngine.loadModel] Input:  " << name << " (index " << i << ")");
        } else {
            outputIndex_ = i;
            FRC_INFO("[TensorRTInferenceEngine.loadModel] Output: " << name << " (index " << i << ")");
        }
    }

    Dims inputDims = engine_->getBindingDimensions(inputIndex_);
    Dims outputDims = engine_->getBindingDimensions(outputIndex_);

    inputSize_ = volume(inputDims) * sizeof(float);
    outputSize_ = volume(outputDims) * sizeof(float);

    FRC_INFO("[TensorRTInferenceEngine.loadModel] Input Size: " << inputSize_ / sizeof(float) << " floats");
    FRC_INFO("[TensorRTInferenceEngine.loadModel] Output Size: " << outputSize_ / sizeof(float) << " floats");

    cudaMalloc(&buffers_[inputIndex_], inputSize_);
    cudaMalloc(&buffers_[outputIndex_], outputSize_);
}

void TensorRTInferenceEngine::warmUp(int rounds){
    FRC_INFO("[TensorRTInferenceEngine.WarmUp] Running " << rounds << " rounds using temporary context...");
    auto tmpContext = std::unique_ptr<IExecutionContext, TRTDestroyer<IExecutionContext>>(engine_->createExecutionContext());
    std::vector<float> dummy(inputSize_ / sizeof(float), 0.01f);

    for (int i = 0; i < rounds; ++i) {
        cudaMemcpy(buffers_[inputIndex_], dummy.data(), inputSize_, cudaMemcpyHostToDevice);
        tmpContext->enqueueV2(buffers_, stream_, nullptr);
        cudaStreamSynchronize(stream_);
        std::vector<float> output(outputSize_ / sizeof(float));
        cudaMemcpy(output.data(), buffers_[outputIndex_], outputSize_, cudaMemcpyDeviceToHost);
        if (i < 3) {
            float avg = std::accumulate(output.begin(), output.end(), 0.0f) / output.size();
            FRC_INFO("[TensorRTInferenceEngine.WarmUp] Round " << i << " avg = " << avg );
        }
    }
}

void TensorRTInferenceEngine::reset(const std::string& method_name){
    context_.reset(engine_->createExecutionContext());
    FRC_INFO("[TensorRTInferenceEngine.Reset] ExecutionContext recreated for method: " << method_name);
}

Eigen::VectorXf TensorRTInferenceEngine::predict(const Eigen::VectorXf& observation) {
    std::vector<float> input(inputSize_ / sizeof(float), 0.0f);

    cudaMemcpy(buffers_[inputIndex_], input.data(), inputSize_, cudaMemcpyHostToDevice);
    context_->enqueueV2(buffers_, stream_, nullptr);
    cudaStreamSynchronize(stream_);

    std::vector<float> output(outputSize_ / sizeof(float));
    cudaMemcpy(output.data(), buffers_[outputIndex_], outputSize_, cudaMemcpyDeviceToHost);

    // return Eigen::Map<Eigen::VectorXf>(acTorch.data_ptr<float>(), acDim);
    return Eigen::VectorXf::Zero(acDim);
}

