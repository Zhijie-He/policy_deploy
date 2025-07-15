// tasks/BaseTask.cpp
#include "tasks/BaseTask.h"
#include "utility/logger.h"
#include "inference_engine/BasePolicyInferenceEngine.h"
#include "inference_engine/PolicyInferenceEngineFactory.h"

BaseTask::BaseTask(std::shared_ptr<const BaseRobotConfig> cfg,
                   std::shared_ptr<const BaseTaskCfg> task_cfg, 
                   torch::Device device,
                   const std::string& hands_type,
                   const std::string& inference_engine_type,
                   const std::string& precision)
    : cfg_(cfg), 
      _kP(cfg->kP),
      _kD(cfg->kD),
      handsDim(cfg->hand_map.at(hands_type).hands_num),
      hands_type_(hands_type),
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
    engine_->warmUp();
    engine_->reset("reset_hist_buffer");
    FRC_HIGHLIGHT("[BaseTask.Const] Policy Inference Engine created with {" << inference_engine_type << "} backend " << "and {" << precision << "} precision!");
  } catch (const std::exception &e) {
      FRC_ERROR("[BaseTask.Const] Failed to create inference engine: " << e.what());
      std::exit(EXIT_FAILURE);
  }
}

void BaseTask::resolveSelfObservation(const CustomTypes::RobotData &raw_obs) {
  // Eigen::Vector3f projected_gravity_b = tools::quat_rotate_inverse_on_gravity(raw_obs.root_rot * task_cfg_->self_obs_scale.at("projected_gravity_b"));
  // Eigen::Vector3f root_ang_vel_b = raw_obs.root_ang_vel * task_cfg_->self_obs_scale.at("root_ang_vel_b");
  // Eigen::VectorXf joint_pos = (raw_obs.joint_pos - cfg_->default_angles) * task_cfg_->self_obs_scale.at("joint_pose");
  // Eigen::VectorXf joint_vel = raw_obs.joint_vel * task_cfg_->self_obs_scale.at("joint_vel");
  // Eigen::VectorXf last_action = actionPrev * task_cfg_->self_obs_scale.at("action");

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

void BaseTask::resolveObservation(const CustomTypes::RobotData &raw_obs) {
  resolveSelfObservation(raw_obs);
  resolveTaskObservation(raw_obs);
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
  
  // 3. clip 动作到合理范围
  for (int i = 0; i < action.size(); ++i) {
      if (action[i] > task_cfg_->action_clip) action[i] = task_cfg_->action_clip;
      if (action[i] < -task_cfg_->action_clip) action[i] = -task_cfg_->action_clip;
  }

  // 4. 构造控制命令
  CustomTypes::Action robotAction = CustomTypes::zeroAction(acDim, handsDim);
  robotAction.timestamp = robotData.timestamp;
  
  robotAction.motorPosition = action(cfg_->actor2env) * cfg_->action_scale + cfg_->default_angles;  // 应用动作映射（比如 env 控制的只有某些 motor）
  robotAction.motorVelocity.setZero();
  robotAction.motorTorque.setZero();

  robotAction.handsPosition.setZero();
  robotAction.handsVelocity.setZero();
  robotAction.handsTorque.setZero();
  // robotAction.kP = _kP;
  // robotAction.kD = _kD;

  // 5. 保存历史动作
  actionPrev = action;
  
  return robotAction;
}

void BaseTask::reset(){
  counter_ = 0;
  start_ = false;
  engine_->reset("reset_hist_buffer");
}

void BaseTask::setVisualization(const std::vector<std::array<float, 3>>& vis) {
  int N = vis.size();  // e.g., 11
  visualization_ = Eigen::MatrixXf::Zero(N, 3);
  for (int i = 0; i < N; ++i) {
      visualization_.row(i) = Eigen::Vector3f(vis[i][0], vis[i][1], vis[i][2]);
  }
}

const Eigen::MatrixXf& BaseTask::getVisualization() const{
  return visualization_;
}

