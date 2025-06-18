#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace CustomTypes {
//    const int jntNum = 28;
//    const int legNum = 2;
    typedef struct State {
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

    typedef struct Action {
        Eigen::VectorXf motorPosition;
        Eigen::VectorXf motorVelocity;
        Eigen::VectorXf motorTorque;
        Eigen::VectorXf kP;
        Eigen::VectorXf kD;
        uint64_t timestamp;
    } Action;

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

