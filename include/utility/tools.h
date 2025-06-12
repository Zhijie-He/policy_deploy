// utility/tools.h

#pragma once
#include <Eigen/Dense>
#include <memory>
#include "config/EmanRobotConfig.h"
#include "config/UnitreeRobotConfig.h"

namespace tools {
    std::shared_ptr<BaseRobotConfig> loadConfig(const std::string& config_name);
    Eigen::VectorXf pd_control(const Eigen::VectorXf& target_q,
                                  const Eigen::VectorXf& q,
                                  const Eigen::VectorXf& kp,
                                  const Eigen::VectorXf& target_dq,
                                  const Eigen::VectorXf& dq,
                                  const Eigen::VectorXf& kd);
}


