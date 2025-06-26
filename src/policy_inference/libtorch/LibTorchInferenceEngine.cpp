// policy_inference/libtorch/LibTorchInferenceEngine.cpp
#include "policy_inference/libtorch/LibTorchInferenceEngine.h"
#include "utility/logger.h"

LibTorchInferenceEngine::LibTorchInferenceEngine(std::shared_ptr<const BaseRobotConfig> cfg,
                                                 torch::Device device,
                                                 const std::string& precision)
    : BasePolicyInferenceEngine(cfg, device, precision)
{
    loadModel();
}

void LibTorchInferenceEngine::loadModel(){
    try {
        module_ = torch::jit::load(cfg_->policy_path);
        module_.to(device_);
        module_.eval();
    } catch (const std::exception& e) {
        FRC_ERROR("[LibTorchInferenceEngine] Failed to load model: " << e.what());
        std::exit(EXIT_FAILURE);
    }
}

void LibTorchInferenceEngine::warmUp(int rounds){
    // Pre-Running for warmup. Otherwise, first running takes more time。
    for (int i = 0; i < rounds; i++) { 
        obTorch = (torch::ones({1, obDim}) + torch::rand({1, obDim}) * 0.01f).to(device_, precision_);
        obVector.clear();
        obVector.emplace_back(obTorch); // 把 obTorch 这个 Tensor 包装成 IValue 并压入 obVector
        acTorch = module_.forward(obVector).toTensor().to(torch::kCPU, torch::kFloat32);  // 推理完可转回 CPU
        if (i < 3) FRC_INFO("[LibTorchInferenceEngine.warmUp] Warm up " << i << "th Action(" << acTorch.sizes() << "): " << *(acTorch.data_ptr<float>() + 1));
    }
}

void LibTorchInferenceEngine::reset(const std::string& method_name){
    if(method_name.empty()) return;

    try{
        FRC_INFO("[LibTorchInferenceEngine.reset] Calling reset method: " << method_name);
        module_.get_method(method_name)({});
    }catch(const std::exception& e){
        FRC_ERROR("[LibTorchInferenceEngine.reset] Failed to call reset method" << method_name <<":" << e.what());
    }
}

Eigen::VectorXf LibTorchInferenceEngine::predict(const Eigen::VectorXf& observation) {
    obTorch = torch::from_blob(const_cast<float*>(observation.data()), {1, obDim})
                                             .clone()
                                             .to(device_, precision_);
    obVector.clear();
    obVector.emplace_back(obTorch);
    acTorch = module_.forward(obVector).toTensor().to(torch::kCPU, torch::kFloat32); // 推理完可转回 CPU
    if (acTorch.sizes() != torch::IntArrayRef({1, acDim})) {
        FRC_ERROR("[LibTorchInferenceEngine.predict] Unexpected output shape: " << acTorch.sizes());
        std::exit(1);
    }
    return Eigen::Map<Eigen::VectorXf>(acTorch.data_ptr<float>(), acDim);
}

