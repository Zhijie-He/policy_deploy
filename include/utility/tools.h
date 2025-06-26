// utility/tools.h

#pragma once
#include <Eigen/Dense>
#include <memory>
#include "config/EmanRobotConfig.h"
#include "config/UnitreeRobotConfig.h"
#include "controller/EmanPolicyWrapper.h"
#include "controller/UnitreePolicyWrapper.h"
#include <mujoco/mujoco.h>
#include <torch/torch.h>
#include "utility/logger.h"

namespace tools {
    std::shared_ptr<BaseRobotConfig> loadConfig(const std::string& config_name);
    std::unique_ptr<BasePolicyWrapper> loadPolicyWrapper(const std::string& config_name, std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device);
    Eigen::VectorXf pd_control(const Eigen::VectorXf& target_q,
                                  const Eigen::VectorXf& q,
                                  const Eigen::VectorXf& kp,
                                  const Eigen::VectorXf& target_dq,
                                  const Eigen::VectorXf& dq,
                                  const Eigen::VectorXf& kd);
    void checkMujucoVersion();
    torch::Device getDefaultDevice(); 
}


