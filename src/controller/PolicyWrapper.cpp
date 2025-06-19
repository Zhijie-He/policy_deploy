#include "controller/UnitreePolicyWrapper.h"
#include <iostream>
#include "utility/logger.h"
#include "utility/timer.h"
#include <chrono>

UnitreePolicyWrapper::UnitreePolicyWrapper(std::shared_ptr<const BaseRobotConfig> cfg):
    NeuralController(cfg)  // 初始化基类
{   
    action.setZero(acDim);
    actionPrev.setZero(acDim);
    observation.setZero(obDim);
    FRC_INFO("[UnitreePolicyWrapper] Constructor Finished.");
}

Eigen::Vector3f get_gravity_orientation(const Eigen::Vector4f& q) {
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];

    Eigen::Vector3f g;
    g[0] = 2 * (-qz * qx + qw * qy);
    g[1] = -2 * (qz * qy + qw * qx);
    g[2] = 1 - 2 * (qw * qw + qz * qz);
    return g;
}

void UnitreePolicyWrapper::updateObservation(const CustomTypes::RobotData &raw_obs) {
    float period = 0.8f;
    float t_sec = raw_obs.timestamp; 
    float phase =  std::fmod(t_sec, period) / period;
    float sin_phase = std::sin(2 * M_PI * phase);
    float cos_phase = std::cos(2 * M_PI * phase);

    Eigen::Vector3f omega = raw_obs.root_ang_vel * cfg_->ang_vel_scale;
    Eigen::Vector3f gravity_orientation = get_gravity_orientation(raw_obs.root_rot);
    Eigen::Vector3f cmd_scaled = raw_obs.targetCMD.cwiseProduct(cfg_->cmd_scale);
    Eigen::VectorXf qj = (raw_obs.joint_pos - cfg_->default_angles) * cfg_->dof_pos_scale;
    Eigen::VectorXf dqj = raw_obs.joint_vel * cfg_->dof_vel_scale;

    // // ---- 构造 observation ----
    observation.segment(0, 3) = omega;
    observation.segment(3, 3) = gravity_orientation;
    observation.segment(6, 3) = cmd_scaled;
    observation.segment(9, acDim) = qj;
    observation.segment(9 + acDim, acDim) = dqj;
    observation.segment(9 + 2 * acDim, acDim) = actionPrev;
    observation(9 + 3 * acDim) = sin_phase;
    observation(9 + 3 * acDim + 1) = cos_phase;
    observation.segment(9 + 3 * acDim, 2) << sin_phase, cos_phase;
}

CustomTypes::Action UnitreePolicyWrapper::getControlAction(const CustomTypes::RobotData &robotData) {
    // 1. 更新观测
    updateObservation(robotData);

    // FRC_INFO("[UnitreePolicyWrapper] observation = " << observation.transpose());
    if (!observation.allFinite()) {
        FRC_ERROR("[UnitreePolicyWrapper] Observation contains nan or inf! Abort.");
        std::exit(1);
    }

    // 2. 构造 Torch 输入
    obTorch = torch::from_blob(observation.data(), {1, obDim}, torch::kFloat32)
              .clone()
              .to(device_);  // 显式迁移到正确设备

    // 3. 推理输出
    auto t_start = std::chrono::high_resolution_clock::now();
    auto output = module_.forward({obTorch}).toTensor().to(torch::kCPU);  // 推理后迁回 CPU
    auto t_end = std::chrono::high_resolution_clock::now();
    

    TORCH_CHECK(output.sizes() == torch::IntArrayRef({1, acDim}), "Unexpected output shape from policy network");
    action = Eigen::Map<Eigen::VectorXf>(output.data_ptr<float>(), acDim);
    
    // 4. 构造控制命令
    CustomTypes::Action robotAction = CustomTypes::zeroAction(acDim);
    robotAction.timestamp = robotData.timestamp;
    robotAction.motorPosition = action * cfg_->action_scale + cfg_->default_angles;
    robotAction.motorVelocity.setZero();  // 如果不使用可省略
    robotAction.motorTorque.setZero();   // 如果不使用可省略
    robotAction.kP = _kP;
    robotAction.kD = _kD;

    // 5. 保存历史动作
    actionPrev = action;

    return robotAction;
}
