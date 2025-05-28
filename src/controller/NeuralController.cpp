#include "controller/NeuralController.h"
#include "utility/logger.h"

NeuralController::NeuralController(std::shared_ptr<const RobotConfig> cfg)
 : cfg_(cfg),
 obDim(cfg->num_obs),
 acDim(cfg->num_actions),
 kNumMotors(cfg->num_actions)
{
  initMembers();
}

void NeuralController::initMembers() {
  bVel_w.setZero();
  bOmg_w.setZero();
  bOri_eu.setZero();
  bOri_rm.setZero();
  bOri_quat.setZero();

  footForce.setZero();

  jTau.setZero(kNumMotors);
  jPos.setZero(kNumMotors);
  jVel.setZero(kNumMotors);
  baseLinVelTarget.setZero();
  baseAngVelTarget.setZero();

  curVelTarg.setZero();
  curOmgTarg.setZero();

  observation.setZero(obDim);
  action.setZero(acDim);
  actionPrev.setZero(acDim);
  
  _kP = cfg_->kP;
  _kD = cfg_->kD;
}

void NeuralController::loadRobotState(const CustomTypes::State &robotState){
  simulationTime = robotState.timestamp;
  bVel_w     = robotState.baseVelocity_w;     // base 线速度（世界坐标系）
  bOmg_w     = robotState.baseRpyRate_w;      // base 角速度（世界坐标系）
  bOri_eu    = robotState.baseRpy;            // base 姿态欧拉角
  bOri_rm    = robotState.baseRotMat;         // base 姿态旋转矩阵
  bOri_quat  = robotState.baseQuat;           // base 姿态四元数

  footForce = robotState.footForce;           // 足端接触力（可用于接触判断）
  jTau      = robotState.motorTorque;         // 电机力矩（当前输出）
  jPos      = robotState.motorPosition;       // 电机角度
  jVel      = robotState.motorVelocity;       // 电机角速度

  baseLinVelTarget = robotState.targetVelocity;  // 想走的线速度（机体坐标系）
  baseAngVelTarget = robotState.targetOmega;     // 想转的角速度（机体坐标系）

  // 坐标变换到世界系目标速度（可选/分析用）
  curVelTarg = bOri_rm * baseLinVelTarget;
  curOmgTarg = bOri_rm * baseAngVelTarget;
}

CustomTypes::Action NeuralController::constructAction(const Eigen::VectorXf &jntPosTarg) const {
  auto robotAction = CustomTypes::zeroAction(kNumMotors);
  robotAction.timestamp = simulationTime;
  robotAction.motorPosition = jntPosTarg;
  robotAction.kP = _kP;
  robotAction.kD = _kD;
  return robotAction;
}

