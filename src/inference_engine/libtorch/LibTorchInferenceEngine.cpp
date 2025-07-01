// inference_enginelibtorch/LibTorchInferenceEngine.cpp
#include "inference_engine/libtorch/LibTorchInferenceEngine.h"
#include "utility/logger.h"
#include "utility/tools.h"

LibTorchInferenceEngine::LibTorchInferenceEngine(std::shared_ptr<const BaseRobotConfig> cfg,
                                                 torch::Device device,
                                                 const std::string& precision)
    : BasePolicyInferenceEngine(cfg, device, precision),
    hiddenDim(cfg->num_hidden),
    device_(device),
    precision_(tools::parseDtype(precision))
{
    loadModel();
    
    if (hiddenDim > 0) {
        prev_hidden_state_ = Eigen::VectorXf::Zero(hiddenDim);
    }
}

void LibTorchInferenceEngine::loadModel(){
    try {
        module_ = torch::jit::load(cfg_->policy_path);
        module_.to(device_);
        module_.eval();
        FRC_INFO("[LibTorchInferenceEngine.loadModel] model loaded: " << cfg_->policy_path);
    } catch (const std::exception& e) {
        FRC_ERROR("[LibTorchInferenceEngine.loadModel] Failed to load model: " << e.what());
        std::exit(EXIT_FAILURE);
    }
}

void LibTorchInferenceEngine::warmUp(int rounds) {
    int input_dim = obDim + hiddenDim;
    FRC_INFO("[LibTorchInferenceEngine.WarmUp] Running " << rounds 
             << " rounds using random inputs... (input_dim=" << input_dim << ")");

    for (int i = 0; i < rounds; i++) {
        // 构造随机输入：[obs | hidden_state]
        Eigen::VectorXf dummy_input = Eigen::VectorXf::Ones(input_dim) 
                                    + Eigen::VectorXf::Random(input_dim) * 0.91f;

        // 转为 tensor
        obTorch = torch::from_blob(dummy_input.data(), {1, input_dim})
                      .clone().to(device_, precision_);

        obVector.clear();
        obVector.emplace_back(obTorch);

        // 推理
        acTorch = module_.forward(obVector).toTensor().to(torch::kCPU, torch::kFloat32);

        if (i < 3) {
            FRC_INFO("[LibTorchInferenceEngine.warmUp] Warm up " << i 
                     << "th Action(" << acTorch.sizes() << "): " 
                     << *(acTorch.data_ptr<float>() + 1));
        }
    }
}

void LibTorchInferenceEngine::reset(const std::string& method_name){
    // 因为之后把中间态都外面维护了 所以可以不用reset了
    // if(method_name.empty()) return;

    // try{
    //     FRC_INFO("[LibTorchInferenceEngine.reset] Calling reset method: " << method_name);
    //     module_.get_method(method_name)({});
    // }catch(const std::exception& e){
    //     FRC_ERROR("[LibTorchInferenceEngine.reset] Failed to call reset method" << method_name <<":" << e.what());
    // }
    // 重置 C++ 侧隐藏状态缓存
    if (hiddenDim > 0)
        prev_hidden_state_.setZero();  // 确保 predict 中拼接的是全零

}

Eigen::VectorXf LibTorchInferenceEngine::predict(const Eigen::VectorXf& observation) {
    Eigen::VectorXf input(obDim + hiddenDim);    // 拼接输入
    input.head(obDim) = observation;

    if (hiddenDim > 0)
        input.tail(hiddenDim) = prev_hidden_state_;

    // 转换为 tensor
    obTorch = torch::from_blob(input.data(), {1, obDim + hiddenDim})
              .clone().to(device_, precision_);
              
    obVector.clear();
    obVector.emplace_back(obTorch);
    
    acTorch = module_.forward(obVector).toTensor().to(torch::kCPU, torch::kFloat32); // 推理完可转回 CPU
    
    // 安全方案：显式克隆，避免悬挂引用
    torch::Tensor output_tensor = acTorch.clone();  // 必须 clone！

    Eigen::VectorXf output = Eigen::Map<Eigen::VectorXf>(
        output_tensor.data_ptr<float>(), output_tensor.size(1));

    Eigen::VectorXf action = output.head(acDim);

    if (hiddenDim > 0)
        prev_hidden_state_ = output.tail(hiddenDim);

    return action;
}

