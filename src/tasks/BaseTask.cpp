// tasks/BaseTask.cpp
#include "tasks/BaseTask.h"
#include "utility/logger.h"
#include "inference_engine/BasePolicyInferenceEngine.h"
#include "inference_engine/PolicyInferenceEngineFactory.h"


BaseTask::BaseTask(std::shared_ptr<const BaseRobotConfig> cfg,
                   std::shared_ptr<const BaseTaskCfg> task_cfg, 
                   torch::Device device,
                   const std::string& inference_engine_type,
                   const std::string& precision)
    : cfg_(cfg), 
      _kP(cfg->kP),
      _kD(cfg->kD),
      task_cfg_(task_cfg),
      obDim(task_cfg->getNumObs()),
      acDim(task_cfg->getNumActions()), 
      counter_(0), 
      start_(false) 
{
  action.setZero(acDim);
  actionPrev.setZero(acDim);
  observation.setZero(obDim);

  try {
    engine_ = PolicyInferenceEngineFactory::create(inference_engine_type,
                                                   cfg,
                                                   task_cfg,
                                                   device,
                                                   precision);
    FRC_INFO("[BaseTask.Const] Inference engine created with {" << inference_engine_type << "} inference backend " << "and {" << precision << "} precision!");
    
    engine_->warmUp();
    engine_->reset("reset_hist_buffer");
    FRC_INFO("[BaseTask.Const] Engine Constructor Finished.");
  } catch (const std::exception &e) {
      FRC_ERROR("[BaseTask.Const] Failed to create inference engine: " << e.what());
      std::exit(EXIT_FAILURE);
  }
}

void BaseTask::updateObservation(const CustomTypes::RobotData &raw_obs) {
    Eigen::Vector3f projected_gravity_b = tools::quat_rotate_inverse_on_gravity(raw_obs.root_rot * cfg_->obs_scale_projected_gravity_b);
    Eigen::Vector3f root_ang_vel_b = raw_obs.root_ang_vel * cfg_->ang_vel_scale;
    Eigen::VectorXf joint_pos = (raw_obs.joint_pos - cfg_->default_angles) * cfg_->dof_pos_scale;
    Eigen::VectorXf joint_vel = raw_obs.joint_vel * cfg_->dof_vel_scale;
    Eigen::VectorXf last_action = actionPrev * cfg_->action_scale;
    // Eigen::Vector3f cmd = raw_obs.targetCMD.cwiseProduct(cfg_->cmd_scale);

    observation.segment(0, 3)                  = projected_gravity_b;
    observation.segment(3, 3)                  = root_ang_vel_b;
    observation.segment(6, acDim)              = joint_pos(cfg_->env2actor);
    observation.segment(6 + acDim, acDim)      = joint_vel(cfg_->env2actor);
    observation.segment(6 + 2 * acDim, acDim)  = actionPrev;
    // observation.segment(6 + 3 * acDim, 3)      = cmd;
}

CustomTypes::Action BaseTask::getAction(const CustomTypes::RobotData &robotData){
  ++counter_;
  
  // 1. parse观测 (update Obs + resovle Obs)
  resolveObservation(robotData);

  if (!observation.allFinite()) {
      FRC_ERROR("[BaseTask.getAction] Observation contains nan or inf! Abort.");
      std::exit(1);
  }

  // 2. 推理输出
  action = engine_->predict(observation);

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

void BaseTask::reset(){
  counter_ = 0;
  start_ = false;
  engine_->reset("reset_hist_buffer");
}
