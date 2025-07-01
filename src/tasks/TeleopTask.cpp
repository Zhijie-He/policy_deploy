// tasks/TeleopTask.cpp
#include "tasks/TeleopTask.h"
#include "utility/logger.h"
#include "utility/tools.h"

TeleopTask::TeleopTask(std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device) 
     : BaseTask(cfg, std::make_shared<TeleopTaskCfg>(), device)
{
    FRC_INFO("[TeleopTask.Const] Created on " << device << ", control_dt=" << control_dt_);
}

void TeleopTask::resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) {
    FRC_INFO("[TeleopTask.resolveKeyboardInput] Key pressed: " << key);
}

void TeleopTask::resolveObservation(const CustomTypes::RobotData &raw_obs) {
    Eigen::Vector3f projected_gravity_b = tools::quat_rotate_inverse_on_gravity(raw_obs.root_rot * cfg_->obs_scale_projected_gravity_b);
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
