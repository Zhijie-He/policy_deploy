#include "controller/NeuralController.h"
#include "utility/logger.h"
#include "utility/tools.h"

NeuralController::NeuralController(std::shared_ptr<const BaseRobotConfig> cfg)
 :  cfg_(cfg),
    obDim(cfg->num_obs),
    acDim(cfg->num_actions),
    _kP(cfg->kP),
    _kD(cfg->kD)
{
    device_ = tools::getDefaultDevice();
    try {
        module_ = torch::jit::load(cfg_->policy_path, device_);
        FRC_INFO("[NeuralController.Const] model loaded from " << cfg_->policy_path);
    } catch (const c10::Error &e) {
        FRC_ERROR("NeuralController failed to load model: " << e.what());
        std::exit(EXIT_FAILURE);
    }
    
    for (int i = 0; i < 10; i++) { // Pre-Running for warmup. Otherwise, first running takes more time。
        obTorch = (torch::ones({1, obDim}) + torch::rand({1, obDim}) * 0.01f).to(device_);
        obVector.clear();
        obVector.emplace_back(obTorch); // 把 obTorch 这个 Tensor 包装成 IValue 并压入 obVector
        acTorch = module_.forward(obVector).toTensor().to(torch::kCPU);  // 推理完可转回 CPU
        if (i < 3) FRC_INFO("[NeuralController.Const] Warm up " << i << "th Action(" << acTorch.sizes() << "): " << *(acTorch.data_ptr<float>() + 1));
    }
}

