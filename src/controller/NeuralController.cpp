#include "controller/NeuralController.h"
#include "utility/logger.h"
#include "utility/tools.h"

NeuralController::NeuralController(std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device)
 :  cfg_(cfg),
    obDim(cfg->num_obs),
    acDim(cfg->num_actions),
    _kP(cfg->kP),
    _kD(cfg->kD),
    device_(device)
{
    
    try {
        module_ = torch::jit::load(cfg_->policy_path, device_);
        FRC_INFO("[NeuralController.Const] model loaded from " << cfg_->policy_path);
    } catch (const c10::Error &e) {
        FRC_ERROR("NeuralController failed to load model: " << e.what());
        std::exit(EXIT_FAILURE);
    }
}

