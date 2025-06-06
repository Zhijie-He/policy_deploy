#include "controller/EmanPolicyWrapper.h"
#include <iostream>
#include "utility/logger.h"
#include "utility/timer.h"

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

void EmanPolicyWrapper::updateObservation(const CustomTypes::RobotData &robotData) {
    Eigen::Vector3f projected_gravity_b = quat_rotate_inverse_on_gravity(robotData.baseQuat * cfg_->obs_scale_projected_gravity_b);
    Eigen::Vector3f root_ang_vel_b = robotData.baseOmega * cfg_->ang_vel_scale;
    Eigen::VectorXf joint_pos = (robotData.jointPosition - cfg_->default_angles) * cfg_->dof_pos_scale;
    Eigen::VectorXf joint_vel = robotData.jointVelocity * cfg_->dof_vel_scale;
    Eigen::VectorXf last_action = actionPrev * cfg_->action_scale;
    Eigen::Vector3f cmd = robotData.targetCMD.cwiseProduct(cfg_->cmd_scale);

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

    // FRC_INFO("[PolicyWrapper] observation = " << observation.transpose());
    if (!observation.allFinite()) {
        FRC_ERROR("[EmanPolicyWrapper] Observation contains nan or inf! Abort.");
        std::exit(1);
    }

    // 2. 构造 Torch 输入
    obTorch = torch::from_blob(observation.data(), {1, obDim}, torch::kFloat32).clone();

    // 3. 推理输出
    auto output = module_.forward({obTorch}).toTensor();  // shape: [1, act_dim]
    TORCH_CHECK(output.sizes() == torch::IntArrayRef({1, acDim}),
                "Unexpected output shape from policy network");
    action = Eigen::Map<Eigen::VectorXf>(output.data_ptr<float>(), acDim);
    
    // 4. clip 动作到合理范围 [-10, 10]
    for (int i = 0; i < action.size(); ++i) {
        if (action[i] > 10.0f) action[i] = 10.0f;
        if (action[i] < -10.0f) action[i] = -10.0f;
    }

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
