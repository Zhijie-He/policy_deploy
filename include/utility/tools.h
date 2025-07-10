// utility/tools.h

#pragma once
#include <Eigen/Dense>
#include <memory>
#include "config/EmanRobotConfig.h"
#include "config/UnitreeRobotConfig.h"
#include <mujoco/mujoco.h>
#include <torch/torch.h>
#include "utility/logger.h"

namespace tools {
    std::shared_ptr<BaseRobotConfig> loadConfig(const std::string& config_name);
    Eigen::VectorXf pd_control(const Eigen::VectorXf& target_q,
                                  const Eigen::VectorXf& q,
                                  const Eigen::VectorXf& kp,
                                  const Eigen::VectorXf& target_dq,
                                  const Eigen::VectorXf& dq,
                                  const Eigen::VectorXf& kd);
    void checkMujucoVersion();
    torch::Device getDefaultDevice(); 
    torch::Dtype parseDtype(const std::string& precision);
    Eigen::Vector3f get_gravity_orientation(const Eigen::Vector4f& q);
    Eigen::Vector3f quat_rotate_inverse_on_gravity(const Eigen::Vector4f& q);
    float getHeadingFromQuat(const Eigen::Vector4f& quat);

    Eigen::VectorXf resolveCompatibilityConcat(const Eigen::VectorXf& state, const Eigen::VectorXi& joint_concat_index);
    std::pair<Eigen::VectorXf, Eigen::VectorXf> resolveCompatibilitySplit(const Eigen::VectorXf& state, 
                                                                          const std::unordered_map<std::string, Eigen::VectorXi>& joint_split_index); 
}


