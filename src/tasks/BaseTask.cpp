// tasks/BaseTask.cpp
#include "tasks/BaseTask.h"
#include "utility/logger.h"
#include "inference_engine/PolicyInferenceEngineFactory.h"

BaseTask::BaseTask(std::shared_ptr<const BaseRobotConfig> cfg,
                   std::shared_ptr<BaseTaskCfg> task_cfg, 
                   torch::Device device)
    : cfg_(cfg), 
      obDim(cfg->num_obs),
      acDim(cfg->num_actions),    
      _kP(cfg->kP),
      _kD(cfg->kD),
      control_dt_(cfg->getPolicyDt()), 
      task_cfg_(task_cfg),
      device_(device), 
      counter_(0), 
      start_(false) 
{
  action.setZero(acDim);
  actionPrev.setZero(acDim);
  observation.setZero(obDim);
  std::string inference_engine_type = "libtorch";
  std::string precision = "fp32";
  
  try {
    engine_ = PolicyInferenceEngineFactory::create(
      inference_engine_type,
      cfg,
      device,
      precision);
    FRC_INFO("[BaseTask.Const] Inference engine created with {" << inference_engine_type << "} inference backend " << "and {" << precision << "} precision!");
    
    engine_->warmUp();
    // engine_->reset("reset_hist_buffer");
    engine_->reset("reset");
    FRC_INFO("[BaseTask.Const] Constructor Finished.");
  } catch (const std::exception &e) {
      FRC_ERROR("[BaseTask.Const] Failed to create inference engine: " << e.what());
      std::exit(EXIT_FAILURE);
  }
}

CustomTypes::Action BaseTask::getAction(const CustomTypes::RobotData &robotData){
  // 1. parse观测
  resolveObservation(robotData);

  if (!observation.allFinite()) {
      FRC_ERROR("[BaseTask.getAction] Observation contains nan or inf! Abort.");
      std::exit(1);
  }

  // 2. 推理输出
  action = engine_->predict(observation);
  // FRC_WARN("[getAction.action]" << action.transpose());
  
  // 3. clip 动作到合理范围 [-10, 10]
  for (int i = 0; i < action.size(); ++i) {
      if (action[i] > 10.0f) action[i] = 10.0f;
      if (action[i] < -10.0f) action[i] = -10.0f;
  }

  // 4. 构造控制命令
  CustomTypes::Action robotAction = CustomTypes::zeroAction(acDim);
  robotAction.timestamp = robotData.timestamp;
  robotAction.motorPosition = action(cfg_->actor2env) * cfg_->action_scale + cfg_->default_angles;  // 应用动作映射（比如 env 控制的只有某些 motor）
  robotAction.motorVelocity.setZero();
  robotAction.motorTorque.setZero();
  robotAction.kP = _kP;
  robotAction.kD = _kD;

  // 5. 保存历史动作
  actionPrev = action;

  return robotAction;
}