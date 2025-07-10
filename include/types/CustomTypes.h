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

    typedef struct Action {
      // 输出给机器人的动作目标：位置、速度、力矩、PD 增益
      Eigen::VectorXf motorPosition;
      Eigen::VectorXf motorVelocity;
      Eigen::VectorXf motorTorque;

      Eigen::VectorXf handsPosition;
      Eigen::VectorXf handsVelocity;
      Eigen::VectorXf handsTorque;
      
      Eigen::VectorXf kP;
      Eigen::VectorXf kD;
      uint64_t timestamp;
    } Action;

    inline Action zeroAction(int jointNum, int handsNum) {
      struct Action ret;
      ret.timestamp = 0;
      ret.motorPosition.setZero(jointNum);
      ret.motorVelocity.setZero(jointNum);
      ret.motorTorque.setZero(jointNum);

      ret.handsPosition.setZero(handsNum);
      ret.handsVelocity.setZero(handsNum);
      ret.handsTorque.setZero(handsNum);
      
      ret.kP.setZero(jointNum + handsNum);
      ret.kD.setZero(jointNum + handsNum);
      return ret;
    }

    // RobotData
    struct RobotData {
      Eigen::Vector3f root_xyz = Eigen::Vector3f::Zero();
      Eigen::Vector4f root_rot = Eigen::Vector4f::Zero();
      Eigen::VectorXf joint_pos;
      Eigen::VectorXf hands_joint_pos;

      Eigen::Vector3f root_vel = Eigen::Vector3f::Zero();
      Eigen::Vector3f root_ang_vel = Eigen::Vector3f::Zero();
      Eigen::VectorXf joint_vel;
      Eigen::VectorXf hands_joint_vel;

      Eigen::VectorXf joint_torques;
      Eigen::VectorXf hands_joint_torques;

      Eigen::Vector3f targetCMD = Eigen::Vector3f::Zero();
      
      float timestamp = 0;

      RobotData(int jointDim = 12, int handsDim = 0) {
        joint_pos           = Eigen::VectorXf::Zero(jointDim);
        joint_vel           = Eigen::VectorXf::Zero(jointDim);
        joint_torques       = Eigen::VectorXf::Zero(jointDim);
        hands_joint_pos     = Eigen::VectorXf::Zero(handsDim);
        hands_joint_vel     = Eigen::VectorXf::Zero(handsDim);
        hands_joint_torques = Eigen::VectorXf::Zero(handsDim);
      }
    };

    inline RobotData zeroData(int jointNum, int handsNum) {
      return RobotData(jointNum, handsNum);
    }

    struct MocapConfig{
      std::string host;
      int port;
      int fps;
    };

    struct VlaConfig{
      std::string path;
    };
};

