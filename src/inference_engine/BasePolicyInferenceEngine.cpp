#include "inference_engine/BasePolicyInferenceEngine.h"
#include "utility/logger.h"

BasePolicyInferenceEngine::BasePolicyInferenceEngine(std::shared_ptr<const BaseRobotConfig> cfg, 
                                                     torch::Device device, 
                                                     const std::string& precision)
 :cfg_(cfg),
  obDim(cfg->num_obs),
  acDim(cfg->num_actions),
  hiddenDim(cfg->num_hidden),
  precision_str_(precision)
{
  if (hiddenDim > 0) {
    prev_hidden_state_ = Eigen::VectorXf::Zero(hiddenDim);
  }
}

void BasePolicyInferenceEngine::reset(const std::string& method_name){
    // 因为之后把中间态都外面维护了 所以可以不用reset了
    // if(method_name.empty()) return;

    // try{
    //     FRC_INFO("[LibTorchInferenceEngine.reset] Calling reset method: " << method_name);
    //     module_.get_method(method_name)({});
    // }catch(const std::exception& e){
    //     FRC_ERROR("[LibTorchInferenceEngine.reset] Failed to call reset method" << method_name <<":" << e.what());
    // }
    // 重置 C++ 侧隐藏状态缓存
    if (hiddenDim > 0){
      prev_hidden_state_.setZero();  
    }
    FRC_INFO("[BasePolicyInferenceEngine.reset] prev_hidden_state_ setting zero!");
}