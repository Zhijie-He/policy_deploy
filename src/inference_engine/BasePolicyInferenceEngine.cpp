#include "inference_engine/BasePolicyInferenceEngine.h"
#include "utility/logger.h"

BasePolicyInferenceEngine::BasePolicyInferenceEngine(std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device, const std::string& precision)
 :cfg_(cfg),
  obDim(cfg->num_obs),
  acDim(cfg->num_actions),
  precision_str_(precision)
{
  
}

