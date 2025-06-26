#include "controller/BasePolicyWrapper.h"
#include "policy_inference/PolicyInferenceEngineFactory.h"
#include "utility/logger.h"
#include "utility/tools.h"

BasePolicyWrapper::BasePolicyWrapper(std::shared_ptr<const BaseRobotConfig> cfg, 
                                     torch::Device device, 
                                     const std::string& inference_engine_type,
                                     const std::string& precision)
 :  cfg_(cfg),
    obDim(cfg->num_obs),
    acDim(cfg->num_actions),
    _kP(cfg->kP),
    _kD(cfg->kD)
{
    action.setZero(acDim);
    actionPrev.setZero(acDim);
    observation.setZero(obDim);

    try {
        engine_ = PolicyInferenceEngineFactory::create(
            inference_engine_type,
            cfg,
            device,
            precision
        );
        FRC_INFO("[BasePolicyWrapper.Const] Inference engine created with {" << inference_engine_type << "} inference backend " << "and {" << precision << "} precision!");
    } catch (const std::exception &e) {
        FRC_ERROR("[BasePolicyWrapper.Const] Failed to create inference engine: " << e.what());
        std::exit(EXIT_FAILURE);
    }
}

