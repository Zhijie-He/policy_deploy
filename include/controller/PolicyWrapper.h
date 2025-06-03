#pragma once
#include <string>
#include "controller/NeuralController.h" 

class PolicyWrapper : public NeuralController {
public:
    PolicyWrapper(std::shared_ptr<const RobotConfig> cfg);
    CustomTypes::Action getControlAction(const CustomTypes::State& robotState);
    void loadRobotState(const CustomTypes::State& robotState) override;

private:
    void updateObservation(const CustomTypes::State& robotState);
    
    Eigen::VectorXf jntPosTarg;
};
