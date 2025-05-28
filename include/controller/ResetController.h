#pragma once

#include "controller/Controller.h"
#include "utility/cubic_interp.h"
#include <cassert>

/* A simple controller that takes care of resetting the robot to a default pose. */
// ResetController 是一个专门用于机器人归位（homing）的控制器，实现了虚基类 Controller 的接口，使用三次插值函数生成一个“从当前状态到目标姿态”的平滑动作轨迹，并输出到电机。
class ResetController : public Controller
{
public:
    ResetController(Eigen::VectorXf finalMotorPosition, unsigned long max_timesteps, float dt);
    ~ResetController() override;
    void reset() override;
    void reset(const Eigen::VectorXf &finalMotorPosition);
    void reset(const Eigen::VectorXf &finalMotorPosition, unsigned long max_timesteps);
    bool isComplete() const override;
    CustomTypes::Action getControlAction(const CustomTypes::State& robotState) override;

private:
    unsigned long _kMaxTimesteps;
    unsigned long _timestep;
    Eigen::VectorXf _kFinalMotorPosition;
    const float _dt;

    CubicInterp<float> _trajectory;
    bool _isFirstVisit;
    void _initializeTrajectory(const CustomTypes::State& robotState);
};
