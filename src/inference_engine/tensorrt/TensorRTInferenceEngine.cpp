// inference_enginetensorrt/tensorrt/TensorRTInferenceEngine.cpp
#include "inference_engine/tensorrt/TensorRTInferenceEngine.h"

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

    FRC_INFO("[TensorRTInferenceEngine.loadModel] Input dtype: " << static_cast<int>(input_dtype_));
    FRC_INFO("[TensorRTInferenceEngine.loadModel] Output dtype: " << static_cast<int>(output_dtype_));

    input_dtype_ = engine_->getBindingDataType(inputIndex_);
    output_dtype_ = engine_->getBindingDataType(outputIndex_);

    Dims inputDims = engine_->getBindingDimensions(inputIndex_);
    Dims outputDims = engine_->getBindingDimensions(outputIndex_);

    inputSize_ = volume(inputDims) * getElementSize(input_dtype_);
    outputSize_ = volume(outputDims) * getElementSize(output_dtype_);
    // prev_hidden_state_ = Eigen::VectorXf::Zero(inputSize_ / getElementSize(input_dtype_) - obDim);

    FRC_INFO("[TensorRTInferenceEngine.loadModel] Input Size: " << inputSize_ / getElementSize(input_dtype_) << " floats");
    FRC_INFO("[TensorRTInferenceEngine.loadModel] Hidden state Size: " << inputSize_ / getElementSize(input_dtype_) - obDim << " floats");
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

Eigen::VectorXf TensorRTInferenceEngine::predict(const Eigen::VectorXf& observation) {
    Eigen::VectorXf input(obDim + hiddenDim);    // 拼接输入
    input.head(obDim) = observation;

    if (hiddenDim > 0)
        input.tail(hiddenDim) = prev_hidden_state_;

    // === 输入处理 ===
    std::vector<float> input_data(input.data(), input.data() + input.size());
    cudaMemcpy(buffers_[inputIndex_], input.data(), inputSize_, cudaMemcpyHostToDevice);

    // === 推理执行 ===
    context_->enqueueV2(buffers_, stream_, nullptr);
    cudaStreamSynchronize(stream_);

   // --- 拷贝输出 ---
    const int total_output_len = outputSize_ / getElementSize(output_dtype_);
    std::vector<float> output_data(total_output_len);
    cudaMemcpy(output_data.data(), buffers_[outputIndex_], outputSize_, cudaMemcpyDeviceToHost);

    // --- 拆分输出 ---
    Eigen::VectorXf output = Eigen::Map<Eigen::VectorXf>(output_data.data(), total_output_len);
    Eigen::VectorXf action = output.head(acDim);
     if (hiddenDim > 0)
        prev_hidden_state_ = output.tail(hiddenDim);
    return action;
}
