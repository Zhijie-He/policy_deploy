// G1Sim2MujocoEnv.cpp
#include "sim2/base_env.h"

BaseEnv::BaseEnv(std::shared_ptr<const BaseRobotConfig> cfg,
          std::shared_ptr<DataBuffer<jointCMD>> jointCMDBufferPtr,
          std::shared_ptr<DataBuffer<robotStatus>> robotStatusBufferPtr)
    : cfg_(cfg),
    jointCMDBufferPtr_(jointCMDBufferPtr),
    robotStatusBufferPtr_(robotStatusBufferPtr),
    control_dt_(cfg->getPolicyDt())
{   

}

void BaseEnv::initState() {
  // ① 机器人状态变量
  gc_.setZero(gcDim_);                //  当前 generalized coordinate（广义坐标，位置）
  gv_.setZero(gvDim_);                //  当前 generalized velocity（广义速度）
  joint_torques_.setZero(jointDim_);  //  当前 tau

  // ② 控制增益
  jointPGain = cfg_->kP;
  jointDGain = cfg_->kD;
  
  // ③ 动作目标
  pTarget.setZero(jointDim_); // desired position（用于位置控制）
  vTarget.setZero(jointDim_); // desired velocity（用于速度控制）

  // ④ 力矩命令
  tauCmd.setZero(jointDim_); //	最终输出的控制力矩（全体）
}

