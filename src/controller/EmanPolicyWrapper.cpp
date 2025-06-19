#include "controller/EmanPolicyWrapper.h"
#include <iostream>
#include "utility/logger.h"
#include "utility/timer.h"
#include <chrono>

EmanPolicyWrapper::EmanPolicyWrapper(std::shared_ptr<const BaseRobotConfig> cfg):
    NeuralController(cfg)  // 初始化基类
{   
    action.setZero(acDim);
    actionPrev.setZero(acDim);
    observation.setZero(obDim);

    for (int i = 0; i < 10; i++) { // Pre-Running for warmup. Otherwise, first running takes more time。
        obTorch = (torch::ones({1, obDim}) + torch::rand({1, obDim}) * 0.01f).to(device_);
        obVector.clear();
        obVector.emplace_back(obTorch); // 把 obTorch 这个 Tensor 包装成 IValue 并压入 obVector
        acTorch = module_.forward(obVector).toTensor().to(torch::kCPU);  // 推理完可转回 CPU
        if (i < 3) FRC_INFO("[NeuralController.Const] Warm up " << i << "th Action(" << acTorch.sizes() << "): " << *(acTorch.data_ptr<float>() + 1));
    }
    module_.get_method("reset_hist_buffer")({});

    FRC_INFO("[EmanPolicyWrapper] Constructor Finished.");
}

Eigen::Vector3f quat_rotate_inverse_on_gravity(const Eigen::Vector4f& q) {
    Eigen::Vector3f v(0.0f, 0.0f, -1.0f);  // world gravity vector

    float qw = q[0];
    Eigen::Vector3f q_vec = q.tail<3>();

    Eigen::Vector3f a = v * (2.0f * qw * qw - 1.0f);
    Eigen::Vector3f b = 2.0f * qw * q_vec.cross(v);
    Eigen::Vector3f c = 2.0f * q_vec * (q_vec.dot(v));

    return a - b + c;
}

void EmanPolicyWrapper::updateObservation(const CustomTypes::RobotData &raw_obs) {
    Eigen::Vector3f projected_gravity_b = quat_rotate_inverse_on_gravity(raw_obs.root_rot * cfg_->obs_scale_projected_gravity_b);
    Eigen::Vector3f root_ang_vel_b = raw_obs.root_ang_vel * cfg_->ang_vel_scale;
    Eigen::VectorXf joint_pos = (raw_obs.joint_pos - cfg_->default_angles) * cfg_->dof_pos_scale;
    Eigen::VectorXf joint_vel = raw_obs.joint_vel * cfg_->dof_vel_scale;
    Eigen::VectorXf last_action = actionPrev * cfg_->action_scale;
    Eigen::Vector3f cmd = raw_obs.targetCMD.cwiseProduct(cfg_->cmd_scale);

    observation.segment(0, 3)                  = projected_gravity_b;
    observation.segment(3, 3)                  = root_ang_vel_b;
    observation.segment(6, acDim)              = joint_pos(cfg_->env2actor);
    observation.segment(6 + acDim, acDim)      = joint_vel(cfg_->env2actor);
    observation.segment(6 + 2 * acDim, acDim)  = actionPrev;
    observation.segment(6 + 3 * acDim, 3)      = cmd;

    // 打印调试信息
    // FRC_INFO("################################################################################");
    // FRC_INFO("[RobotData] raw_obs.root_xyz: " << raw_obs.root_xyz.transpose());
    // FRC_INFO("[RobotData] raw_obs.root_rot: " << raw_obs.root_rot.transpose());
    // FRC_INFO("[RobotData] raw_obs.joint_pos: " << raw_obs.joint_pos.transpose());
    // FRC_INFO("[RobotData] raw_obs.root_vel: " << raw_obs.root_vel.transpose());
    // FRC_INFO("[RobotData] raw_obs.root_ang_vel: " << raw_obs.root_ang_vel.transpose());
    // FRC_INFO("[RobotData] raw_obs.joint_vel: " << raw_obs.joint_vel.transpose());
    // FRC_INFO("[RobotData] raw_obs.joint_torques: " << raw_obs.joint_torques.transpose());
    // FRC_INFO("[RobotData] raw_obs.targetCMD: " << raw_obs.targetCMD.transpose());
    
    // FRC_INFO("[Obs] projected_gravity_b (0-2): " << projected_gravity_b.transpose());
    // FRC_INFO("[Obs] root_ang_vel_b      (3-5): " << root_ang_vel_b.transpose());
    // FRC_INFO("[Obs] joint_pos[env2actor] (6-" << 6 + acDim - 1 << "): " << joint_pos(cfg_->env2actor).transpose());
    // FRC_INFO("[Obs] joint_vel[env2actor] (" << 6 + acDim << "-" << 6 + 2 * acDim - 1 << "): " << joint_vel(cfg_->env2actor).transpose());
    // FRC_INFO("[Obs] actionPrev          (" << 6 + 2 * acDim << "-" << 6 + 3 * acDim - 1 << "): " << actionPrev.transpose());
    // FRC_INFO("[Obs] cmd                 (" << 6 + 3 * acDim << "-" << 6 + 3 * acDim + 2 << "): " << cmd.transpose());
    // FRC_INFO("-------------------------------------------------------------------");
}

CustomTypes::Action EmanPolicyWrapper::getControlAction(const CustomTypes::RobotData &robotData) {
    // 1. 更新观测
    updateObservation(robotData);

    // FRC_INFO("[EmanPolicyWrapper] observation = " << observation.transpose());
    if (!observation.allFinite()) {
        FRC_ERROR("[EmanPolicyWrapper] Observation contains nan or inf! Abort.");
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

    // 4. clip 动作到合理范围 [-10, 10]
    for (int i = 0; i < action.size(); ++i) {
        if (action[i] > 10.0f) action[i] = 10.0f;
        if (action[i] < -10.0f) action[i] = -10.0f;
    }

    // 5. 构造控制命令
    CustomTypes::Action robotAction = CustomTypes::zeroAction(acDim);
    robotAction.timestamp = robotData.timestamp;
    robotAction.motorPosition = action(cfg_->actor2env) * cfg_->action_scale + cfg_->default_angles;  // 应用动作映射（比如 env 控制的只有某些 motor）
    robotAction.motorVelocity.setZero();
    robotAction.motorTorque.setZero();
    robotAction.kP = _kP;
    robotAction.kD = _kD;

    // 6. 保存历史动作
    actionPrev = action;

    return robotAction;
}
