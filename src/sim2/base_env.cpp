// ===== src/sim2/BaseEnv.cpp =====
#include "sim2/base_env.h"

 BaseEnv::BaseEnv(std::shared_ptr<const BaseRobotConfig> cfg,
                  std::shared_ptr<StateMachine> state_machine)
      : cfg_(cfg),
        state_machine_(state_machine),
        jointCMDBufferPtr_(state_machine->getJointCMDBufferPtr()),
        robotStatusBufferPtr_(state_machine->getRobotStatusBufferPtr()),
        control_dt_(cfg->getPolicyDt())
{

}

void BaseEnv::initState() {
  gcDim_    = cfg_->num_actions + 7; 
  gvDim_    = cfg_->num_actions + 6; 
  jointDim_ = cfg_->num_actions;
  actuatorDim_ = jointDim_; 

  // ① 机器人状态变量
  gc_.setZero(gcDim_);      //  当前 generalized coordinate（广义坐标，位置）
  gv_.setZero(gvDim_);      //  当前 generalized velocity（广义速度）

  // ② 控制增益
  jointPGain = cfg_->kP;
  jointDGain = cfg_->kD;

  // ③ 动作目标
  pTarget.setZero(actuatorDim_);
  vTarget.setZero(actuatorDim_); // desired velocity（用于速度控制）

  // ④ 力矩命令
  tauCmd.setZero(actuatorDim_); //	最终输出的控制力矩（全体）
}
