// policy_inference/tensorrt/TensorRTInferenceEngine.cpp
#include "policy_inference/tensorrt/TensorRTInferenceEngine.h"

inline size_t getElementSize(nvinfer1::DataType dtype) {
    switch (dtype) {
        case nvinfer1::DataType::kFLOAT: return 4;  // float32
        case nvinfer1::DataType::kHALF:  return 2;  // float16
        case nvinfer1::DataType::kINT8:  return 1;
        case nvinfer1::DataType::kINT32: return 4;
        default: throw std::runtime_error("Unsupported DataType");
    }
}

TensorRTInferenceEngine::TensorRTInferenceEngine(std::shared_ptr<const BaseRobotConfig> cfg,
                                                 torch::Device device,
                                                 const std::string& precision)
    : BasePolicyInferenceEngine(cfg, device, precision)
{
    loadModel();
}

void TensorRTInferenceEngine::loadModel(){
    std::vector<char> engineData = loadEngineFile(cfg_->engine_path);
    FRC_INFO("[TensorRTInferenceEngine.loadModel] Load engine file:  " << cfg_->engine_path);

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

    FRC_INFO("[TensorRTInferenceEngine.loadModel] Input binding:  " << engine_->getBindingName(inputIndex_)
         << " | Type: "
         << (input_dtype_ == DataType::kFLOAT ? "FP32" :
             input_dtype_ == DataType::kHALF  ? "FP16" : "Other"));

    FRC_INFO("[TensorRTInferenceEngine.loadModel] Output binding: " << engine_->getBindingName(outputIndex_)
            << " | Type: "
            << (output_dtype_ == DataType::kFLOAT ? "FP32" :
                output_dtype_ == DataType::kHALF  ? "FP16" : "Other"));

    input_dtype_ = engine_->getBindingDataType(inputIndex_);
    output_dtype_ = engine_->getBindingDataType(outputIndex_);

    Dims inputDims = engine_->getBindingDimensions(inputIndex_);
    Dims outputDims = engine_->getBindingDimensions(outputIndex_);

    inputSize_ = volume(inputDims) * getElementSize(input_dtype_);
    outputSize_ = volume(outputDims) * getElementSize(output_dtype_);

    FRC_INFO("[TensorRTInferenceEngine.loadModel] Input Size: " << inputSize_ / getElementSize(input_dtype_) << " floats");
    FRC_INFO("[TensorRTInferenceEngine.loadModel] Output Size: " << outputSize_ / getElementSize(output_dtype_) << " floats");

    cudaMalloc(&buffers_[inputIndex_], inputSize_);
    cudaMalloc(&buffers_[outputIndex_], outputSize_);
}

void TensorRTInferenceEngine::warmUp(int rounds) {
    FRC_INFO("[TensorRTInferenceEngine.WarmUp] Running " << rounds << " rounds using temporary context...");
    auto tmpContext = std::unique_ptr<IExecutionContext, TRTDestroyer<IExecutionContext>>(engine_->createExecutionContext());

    // === 构造 dummy 输入 ===
    if (input_dtype_ == DataType::kFLOAT) {
        std::vector<float> dummy(inputSize_ / sizeof(float), 0.01f);
        cudaMemcpy(buffers_[inputIndex_], dummy.data(), inputSize_, cudaMemcpyHostToDevice);
    } else if (input_dtype_ == DataType::kHALF) {
        std::vector<__half> dummy(inputSize_ / sizeof(__half));
        for (auto& val : dummy) val = __float2half(0.01f);
        cudaMemcpy(buffers_[inputIndex_], dummy.data(), dummy.size() * sizeof(__half), cudaMemcpyHostToDevice);
    } else {
        throw std::runtime_error("Unsupported input type in warmup");
    }

    // === Warm-up loop ===
    for (int i = 0; i < rounds; ++i) {
        tmpContext->enqueueV2(buffers_, stream_, nullptr);
        cudaStreamSynchronize(stream_);

        float avg = 0.0f;

        if (output_dtype_ == DataType::kFLOAT) {
            std::vector<float> output(outputSize_ / sizeof(float));
            cudaMemcpy(output.data(), buffers_[outputIndex_], output.size() * sizeof(float), cudaMemcpyDeviceToHost);
            if (i < 3) {
                avg = std::accumulate(output.begin(), output.end(), 0.0f) / output.size();
            }
        } else if (output_dtype_ == DataType::kHALF) {
            std::vector<__half> output_fp16(outputSize_ / sizeof(__half));
            cudaMemcpy(output_fp16.data(), buffers_[outputIndex_], output_fp16.size() * sizeof(__half), cudaMemcpyDeviceToHost);
            if (i < 3) {
                for (auto val : output_fp16) {
                    avg += __half2float(val);
                }
                avg /= output_fp16.size();
            }
        } else {
            throw std::runtime_error("Unsupported output type in warmup");
        }

        if (i < 3) {
            FRC_INFO("[TensorRTInferenceEngine.WarmUp] Round " << i << " avg = " << avg);
        }
    }
}


void TensorRTInferenceEngine::reset(const std::string& method_name){
    context_.reset(engine_->createExecutionContext());
    FRC_INFO("[TensorRTInferenceEngine.Reset] ExecutionContext recreated for method: " << method_name);
}

Eigen::VectorXf TensorRTInferenceEngine::predict(const Eigen::VectorXf& observation) {
    // === 输入处理 ===
    std::vector<float> input(inputSize_ / getElementSize(input_dtype_), 0.0f);
    if(input_dtype_ == DataType::kFLOAT){
        cudaMemcpy(buffers_[inputIndex_], input.data(), inputSize_, cudaMemcpyHostToDevice);
    } else if (input_dtype_ == DataType::kHALF) {
        // FP16 输入需要转换
        std::vector<__half> input_fp16(input.size());
        for (int i = 0; i < input.size(); ++i) {
            input_fp16[i] = __float2half(input[i]);
        }
        cudaMemcpy(buffers_[inputIndex_], input_fp16.data(), input_fp16.size() * sizeof(__half), cudaMemcpyHostToDevice);
    }
    else {
        throw std::runtime_error("Unsupported input data type");
    }

    // === 推理执行 ===
    context_->enqueueV2(buffers_, stream_, nullptr);
    cudaStreamSynchronize(stream_);

    // === 输出处理 ===
    Eigen::VectorXf result(outputSize_ / getElementSize(output_dtype_));
    if (output_dtype_ == DataType::kFLOAT) {
        std::vector<float> output(outputSize_ / sizeof(float));
        cudaMemcpy(output.data(), buffers_[outputIndex_], output.size() * sizeof(float), cudaMemcpyDeviceToHost);
        result = Eigen::Map<Eigen::VectorXf>(output.data(), output.size());
    } else if (output_dtype_ == DataType::kHALF) {
        std::vector<__half> output_fp16(outputSize_ / sizeof(__half));
        cudaMemcpy(output_fp16.data(), buffers_[outputIndex_], output_fp16.size() * sizeof(__half), cudaMemcpyDeviceToHost);
        for (int i = 0; i < outputSize_ / sizeof(float); ++i) {
            result[i] = __half2float(output_fp16[i]);
        }
    } else {
        throw std::runtime_error("Unsupported output data type");
    }

    return Eigen::VectorXf::Zero(acDim);
}

