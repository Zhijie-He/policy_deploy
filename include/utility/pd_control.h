// utility/pd_control.h
#pragma once
#include <Eigen/Dense>

inline Eigen::VectorXf pd_control(const Eigen::VectorXf& target_q,
                                  const Eigen::VectorXf& q,
                                  const Eigen::VectorXf& kp,
                                  const Eigen::VectorXf& target_dq,
                                  const Eigen::VectorXf& dq,
                                  const Eigen::VectorXf& kd) {
    return (target_q - q).cwiseProduct(kp) + (target_dq - dq).cwiseProduct(kd);
}
