#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace CustomTypes {
    typedef struct State {
      // 机器当前状态：位置、速度、旋转矩阵、IMU 数据、关节状态、目标速度等
      Eigen::Vector3f basePosition;
      Eigen::Vector3f baseVelocity;
      Eigen::Matrix3f baseRotMat;
      Eigen::Vector3f baseAcc;
      Eigen::Vector3f baseRpy;
      Eigen::Vector4f baseQuat;
      Eigen::Vector3f baseRpyRate;
      Eigen::Matrix<float, 3, Eigen::Dynamic> footForce;
      Eigen::VectorXf motorPosition;
      Eigen::VectorXf motorVelocity;
      Eigen::VectorXf motorTorque;
      Eigen::Vector3f baseVelocity_w;
      Eigen::Vector3f baseRpyRate_w;
      Eigen::Vector3f targetVelocity;
      Eigen::Vector3f targetOmega;
      uint64_t timestamp;
      float phaseValue;
    } State;

    inline std::ostream& operator<<(std::ostream& os, const State& s) {
        os << "[State] "
          << "t=" << s.timestamp << ", phase=" << s.phaseValue << "\n"
          << "  pos=" << s.basePosition.transpose()
          << ", vel=" << s.baseVelocity.transpose() << "\n"
          << "  rpy=" << s.baseRpy.transpose()
          << ", quat=" << s.baseQuat.transpose() << "\n"
          << "  target_vel=" << s.targetVelocity.transpose()
          << ", target_omega=" << s.targetOmega.transpose();
        return os;
    }

    typedef struct Action {
      // 输出给机器人的动作目标：位置、速度、力矩、PD 增益
      Eigen::VectorXf motorPosition;
      Eigen::VectorXf motorVelocity;
      Eigen::VectorXf motorTorque;
      Eigen::VectorXf kP;
      Eigen::VectorXf kD;
      uint64_t timestamp;
    } Action;

    inline std::ostream& operator<<(std::ostream& os, const Action& a) {
        os << "[Action] t=" << a.timestamp << "\n"
          << "  pos=" << a.motorPosition.transpose()
          << ", vel=" << a.motorVelocity.transpose()
          << ", torque=" << a.motorTorque.transpose();
        return os;
    }

    inline State zeroState(int jntNum, int legNum=2) {
      struct State ret;
      ret.phaseValue = 0.;
      ret.timestamp = 0;
      ret.baseRotMat.setZero();
      ret.baseRpy.setZero();
      ret.baseAcc.setZero();
      ret.baseQuat.setZero();
      ret.basePosition.setZero();
      ret.baseVelocity.setZero();
      ret.baseRpyRate.setZero();
      ret.baseVelocity_w.setZero();
      ret.baseRpyRate_w.setZero();
      ret.footForce.setZero();
      ret.targetVelocity.setZero();
      ret.targetOmega.setZero();
      ret.footForce.setZero(3, legNum);
      ret.motorPosition.setZero(jntNum);
      ret.motorVelocity.setZero(jntNum);
      ret.motorTorque.setZero(jntNum);
      return ret;
    }

    inline Action zeroAction(int jntNum) {
      struct Action ret;
      ret.timestamp = 0;
      ret.motorPosition.setZero(jntNum);
      ret.motorVelocity.setZero(jntNum);
      ret.motorTorque.setZero(jntNum);
      ret.kP.setZero(jntNum);
      ret.kD.setZero(jntNum);
      return ret;
    }
};

