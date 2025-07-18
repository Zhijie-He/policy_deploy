#include "inference_engine/BasePolicyInferenceEngine.h"
#include "utility/logger.h"

BasePolicyInferenceEngine::BasePolicyInferenceEngine(std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device, const std::string& precision)
 :cfg_(cfg),
  obDim(cfg->num_obs),
  acDim(cfg->num_actions),
  hiddenDim(cfg->num_hidden),
  policy_path_(cfg->policy_path),
  engine_path_(cfg->engine_path),
  precision_str_(precision)
{
  if (hiddenDim > 0) {
    prev_hidden_state_ = Eigen::VectorXf::Zero(hiddenDim);
  }
}

void BasePolicyInferenceEngine::reset(const std::string& method_name){
    // 重置 C++ 侧隐藏状态缓存
    if (hiddenDim > 0){
      prev_hidden_state_.setZero();  
    }
    FRC_INFO("[BasePolicyInferenceEngine.reset] prev_hidden_state_ setting zero!");
}


