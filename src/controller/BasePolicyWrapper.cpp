#include "controller/BasePolicyWrapper.h"
#include "policy_inference/PolicyInferenceEngineFactory.h"
#include "utility/logger.h"
#include "utility/tools.h"

BasePolicyWrapper::BasePolicyWrapper(std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device)
 :  cfg_(cfg),
    obDim(cfg->num_obs),
    acDim(cfg->num_actions),
    _kP(cfg->kP),
    _kD(cfg->kD),
    device_(device)
{
    action.setZero(acDim);
    actionPrev.setZero(acDim);
    observation.setZero(obDim);

    try {
        engine_ = PolicyInferenceEngineFactory::create(
            "libtorch",
            cfg_->policy_path,
            device,
            "fp32"
        );
        FRC_INFO("[BasePolicyWrapper] Inference engine created with backend");
        FRC_INFO("[BasePolicyWrapper] engine_ pointer: " << engine_.get());
    } catch (const std::exception &e) {
        FRC_ERROR("[BasePolicyWrapper] Failed to create inference engine: " << e.what());
        std::exit(EXIT_FAILURE);
    }
    module_ = engine_->module_;
    
    // try {
    //     module_ = torch::jit::load(cfg_->policy_path, device_);
    //     FRC_INFO("[BasePolicyWrapper.Const] model loaded from " << cfg_->policy_path);
    // } catch (const c10::Error &e) {
    //     FRC_ERROR("BasePolicyWrapper failed to load model: " << e.what());
    //     std::exit(EXIT_FAILURE);
    // }
}

