#pragma once

#include "core/RobotConfig.h" 
#include "types/CustomTypes.h"
#include "types/cpp_types.h"
#include <torch/script.h>

/*  Controller that performs neural net inference */
class NeuralController
{
public:
    NeuralController(std::shared_ptr<const RobotConfig> cfg);

protected:
    void initMembers();
    void loadRobotState(const CustomTypes::State& robotState);
    CustomTypes::Action constructAction(const Eigen::VectorXf& jntPosTarg) const;
    
    std::shared_ptr<const RobotConfig> cfg_; 
    torch::jit::script::Module module_;
    
    size_t kNumMotors = 28;
    int obDim, acDim;
    torch::Tensor obTorch, acTorch;
    std::vector<c10::IValue> obVector{}; // 创建一个 IValue 容器 obVector

    uint64_t simulationTime{0};
    Vec3<float> bVel_w, bOmg_w, bOri_eu;
    Mat3<float> bOri_rm;
    Vec4<float> bOri_quat;
    
    Eigen::Matrix<float, 3, 2> footForce;
    Eigen::VectorXf jTau, jPos, jVel;

    Vec3<float> baseLinVelTarget, baseAngVelTarget;

    Vec3<float> curVelTarg, curOmgTarg;

    Eigen::VectorXf observation;
    Eigen::VectorXf action, actionPrev;

    Eigen::VectorXf _kP;
    Eigen::VectorXf _kD;
};