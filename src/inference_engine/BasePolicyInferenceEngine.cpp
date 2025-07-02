#include "inference_engine/BasePolicyInferenceEngine.h"
#include "utility/logger.h"
#include "tasks/BaseTask.h" 

BasePolicyInferenceEngine::BasePolicyInferenceEngine(std::shared_ptr<const BaseRobotConfig> cfg, 
                                                     std::shared_ptr<const BaseTaskCfg> task_cfg, 
                                                     torch::Device device, 
                                                     const std::string& precision)
 :cfg_(cfg),
  task_cfg_(task_cfg),
  obDim(task_cfg->getNumObs()),
  acDim(task_cfg->getNumActions()),
  hiddenDim(task_cfg->getNumHidden()),
  policy_path_(task_cfg->getPolicyPath()),
  engine_path_(task_cfg->getEnginePath()),
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