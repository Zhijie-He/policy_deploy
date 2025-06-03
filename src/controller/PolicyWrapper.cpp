#include "controller/PolicyWrapper.h"
#include <iostream>
#include "utility/logger.h"
#include "utility/timer.h"

PolicyWrapper::PolicyWrapper(std::shared_ptr<const RobotConfig> cfg):
    NeuralController(cfg)
{   
    // torch::set_num_threads(4); //  LibTorch 在执行张量运算、模型推理时最多使用 4 个线程（并行加速）
    // LibTorch 默认使用多线程（基于 OpenMP 或 native thread pool），线程数可能等于你的 CPU 核心数。
    obDim = cfg->num_obs;
    acDim = cfg->num_actions;
    obTorch = torch::zeros({1, obDim});

    try {
        module_ = torch::jit::load(cfg_->policy_path);
        FRC_INFO("NeuralController: model loaded from " << cfg_->policy_path);
    } catch (const c10::Error &e) {
        FRC_ERROR("NeuralController failed to load model: " << e.what());
        std::exit(EXIT_FAILURE);
    }
    
    for (int i = 0; i < 10; i++) { // Pre-Running for warmup. Otherwise, first running takes more time
        obTorch = torch::ones({1, obDim}) + (i > 50) * torch::rand({1, obDim});   // torch::ones({1, obDim})：构造一个假的全 1 输入。
        obVector.clear();
        obVector.emplace_back(obTorch); // 把 obTorch 这个 Tensor 包装成 IValue 并压入 obVector
        acTorch = module_.forward(obVector).toTensor(); // 将其作为输入送入模型的 forward() 方法
        if (i < 3) // i < 3：只打印前 3 次 warm-up 的输出，便于验证模型行为。
        FRC_INFO("Warm up " << i << "th Action(" << acTorch.sizes() << "): " << *(acTorch.data_ptr<float>() + 1));
    }

    FRC_INFO("[PolicyWrapper] jointNum: " << kNumMotors);
    jntPosTarg.setZero(kNumMotors);
    actionPrev.setZero();

    FRC_INFO("[PolicyWrapper] Constructor Finished.");
}

Eigen::Vector3f getGravityOrientation(const Eigen::Vector4f& quat) {
    float qw = quat[0];
    float qx = quat[1];
    float qy = quat[2];
    float qz = quat[3];

    Eigen::Vector3f gravity_orientation;
    gravity_orientation[0] = 2.0f * (-qz * qx + qw * qy);
    gravity_orientation[1] = -2.0f * (qz * qy + qw * qx);
    gravity_orientation[2] = 1.0f - 2.0f * (qw * qw + qz * qz);

    return gravity_orientation;
}

void PolicyWrapper::loadRobotState(const CustomTypes::State &robotState){
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

void PolicyWrapper::updateObservation(const CustomTypes::State &robotState) {
  loadRobotState(robotState);
  float t_sec = simulationTime / 1e6f; // 这行代码是把变量 simulationTime 从 微秒（microseconds） 转换为 秒（seconds），因为：
  float phasePeriod_ = 0.8f;  // 周期值，和 Python 中一致
  float phase = std::fmod(t_sec, phasePeriod_) / phasePeriod_;
  float sin_phase = std::sin(2 * M_PI * phase);
  float cos_phase = std::cos(2 * M_PI * phase);

  observation.setZero(obDim);
  Eigen::Vector3f omega = bOmg_w * cfg_->ang_vel_scale;
  
  Eigen::Vector3f gravity = getGravityOrientation(bOri_quat);
  Eigen::Vector3f cmd_scaled = baseLinVelTarget.cwiseProduct(cfg_->cmd_scale);
  Eigen::VectorXf qj = (jPos - cfg_->default_angles) * cfg_->dof_pos_scale;
  Eigen::VectorXf dqj = jVel * cfg_->dof_vel_scale;

  observation.segment(0, 3) = omega; // base的角速度
  observation.segment(3, 3) = gravity;
  observation.segment(6, 3) = cmd_scaled;
  observation.segment(9, acDim) = qj;
  observation.segment(9 + acDim, acDim) = dqj;
  observation.segment(9 + 2 * acDim, acDim) = actionPrev;
  observation(9 + 3 * acDim) = sin_phase;
  observation(9 + 3 * acDim + 1) = cos_phase;
}

CustomTypes::Action PolicyWrapper::getControlAction(const CustomTypes::State &robotState) {
    updateObservation(robotState);
    // 1. 创建 Torch 输入
    obTorch = torch::from_blob(observation.data(), {1, obDim}, torch::kFloat32).clone();
    obVector.clear();
    obVector.emplace_back(obTorch);

    // 2. 推理
    acTorch = module_.forward(obVector).toTensor();  // [1, act_dim]
    Eigen::Map<Eigen::VectorXf> rawAction(acTorch.data_ptr<float>(), acDim);
    action = rawAction;  // 保存 action（未还原 scale）

    // 3. 构造控制命令
    jntPosTarg.setZero();
    jntPosTarg = action * cfg_->action_scale + cfg_->default_angles;
    auto acStruct = constructAction(jntPosTarg);
    
    // 4. 保存上一个动作用于下次 obs 构造
    actionPrev = action;

    return acStruct;
}
