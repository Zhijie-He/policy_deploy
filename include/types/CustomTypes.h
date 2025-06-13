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
        os << "[State Dump]\n";
        os << "  ── Time Stamp      : " << s.timestamp << "\n";
        os << "  ── Phase Value     : " << s.phaseValue << "\n";

        os << "\n  ▸ Base State\n";
        os << "    Base Position    : " << s.basePosition.transpose() << "\n";
        os << "    Base Velocity    : " << s.baseVelocity.transpose() << "\n";
        os << "    Base Velocity (W): " << s.baseVelocity_w.transpose() << "\n";
        os << "    Base Acc         : " << s.baseAcc.transpose() << "\n";
        os << "    Base Orientation : (quat) " << s.baseQuat.transpose() << "\n";
        os << "    Base Orientation : (RPY)  " << s.baseRpy.transpose() << "\n";
        os << "    RPY Rate (body)  : " << s.baseRpyRate.transpose() << "\n";
        os << "    RPY Rate (world) : " << s.baseRpyRate_w.transpose() << "\n";
        os << "    Base Rot Mat     : \n" << s.baseRotMat << "\n";

        os << "\n  ▸ Motor State\n";
        os << "    Motor Position   : " << s.motorPosition.transpose() << "\n";
        os << "    Motor Velocity   : " << s.motorVelocity.transpose() << "\n";
        os << "    Motor Torque     : " << s.motorTorque.transpose() << "\n";

        os << "\n  ▸ Foot Contact (Force)\n";
        os << "    Foot Force Matrix: \n" << s.footForce << "\n";

        os << "\n  ▸ Target Command\n";
        os << "    Target Velocity  : " << s.targetVelocity.transpose() << "\n";
        os << "    Target Omega     : " << s.targetOmega.transpose() << "\n";

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

    // RobotData
    struct RobotData {
      Eigen::Vector3f root_xyz = Eigen::Vector3f::Zero();
      Eigen::Vector4f root_rot = Eigen::Vector4f::Zero();
      Eigen::VectorXf joint_pos;

      Eigen::Vector3f root_vel = Eigen::Vector3f::Zero();
      Eigen::Vector3f root_ang_vel = Eigen::Vector3f::Zero();
      Eigen::VectorXf joint_vel;

      Eigen::VectorXf joint_torques;

      Eigen::Vector3f targetCMD = Eigen::Vector3f::Zero();
      
      float timestamp = 0;

      RobotData(int jointDim = 12) {
        joint_pos = Eigen::VectorXf::Zero(jointDim);
        joint_vel = Eigen::VectorXf::Zero(jointDim);
        joint_torques = Eigen::VectorXf::Zero(jointDim);
      }
    };

    inline RobotData zeroData(int jointNum) {
      return RobotData(jointNum);
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

