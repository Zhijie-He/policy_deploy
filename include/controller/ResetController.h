#pragma once
#include "core/RobotConfig.h" 
#include "utility/cubic_interp.h"
#include <cassert>
#include "types/CustomTypes.h"

/* A simple controller that takes care of resetting the robot to a default pose. */
class ResetController
{
public:
    ResetController(std::shared_ptr<const RobotConfig> cfg);
    ~ResetController();
    void reset();
    void reset(const Eigen::VectorXf &finalMotorPosition);
    void reset(const Eigen::VectorXf &finalMotorPosition, unsigned long max_timesteps);
    bool isComplete() const;
    CustomTypes::Action getControlAction(const CustomTypes::State& robotState);
    
private:
    unsigned long _kMaxTimesteps;
    unsigned long _timestep;
    Eigen::VectorXf _kFinalMotorPosition;
    const float _dt;

    CubicInterp<float> _trajectory;
    bool _isFirstVisit;
    void _initializeTrajectory(const CustomTypes::State& robotState);

    CustomTypes::State _robotState;
    int _jointNum = 28;
    Eigen::VectorXf _kP;
    Eigen::VectorXf _kD;
};
