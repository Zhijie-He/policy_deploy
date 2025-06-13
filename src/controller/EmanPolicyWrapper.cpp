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
    obTorch = torch::from_blob(observation.data(), {1, obDim}, torch::kFloat32).clone();

    // 3. 推理输出
    // auto output = module_.forward({obTorch}).toTensor();  // shape: [1, act_dim]
    auto t_start = std::chrono::high_resolution_clock::now();
    auto output = module_.forward({obTorch}).toTensor();
    auto t_end = std::chrono::high_resolution_clock::now();
    
    double infer_time_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();
    infer_sum_us += infer_time_us;
    infer_sum_sq_us += infer_time_us * infer_time_us;
    ++infer_count;

    if (infer_count % 100 == 0) {
        double avg = infer_sum_us / infer_count;
        double stddev = std::sqrt(infer_sum_sq_us / infer_count - avg * avg);
        // std::cout << "[EmanPolicyWrapper.getControlAction] Inference AVG: " << avg << " us | STDDEV: " << stddev << " us\n";
    }

    TORCH_CHECK(output.sizes() == torch::IntArrayRef({1, acDim}), "Unexpected output shape from policy network");
    action = Eigen::Map<Eigen::VectorXf>(output.data_ptr<float>(), acDim);

    // 4. clip 动作到合理范围 [-10, 10]
    for (int i = 0; i < action.size(); ++i) {
        if (action[i] > 10.0f) action[i] = 10.0f;
        if (action[i] < -10.0f) action[i] = -10.0f;
    }
    // std::cout << "[EmanPolicyWrapper.getControlAction] robotData: " << robotData.jointPosition.transpose()  << std::endl;
    // std::cout << "[EmanPolicyWrapper.getControlAction] action: " << action.transpose()  << std::endl;
    
    // 5. 构造控制命令
    CustomTypes::Action robotAction = CustomTypes::zeroAction(acDim);
    robotAction.timestamp = robotData.timestamp;
    // 应用动作映射（比如 env 控制的只有某些 motor）
    robotAction.motorPosition = action(cfg_->actor2env) * cfg_->action_scale + cfg_->default_angles;
    robotAction.motorVelocity.setZero();
    robotAction.motorTorque.setZero();
    robotAction.kP = _kP;
    robotAction.kD = _kD;

    // 6. 保存历史动作
    actionPrev = action;

    return robotAction;
}
