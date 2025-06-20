#pragma once
#include "controller/NeuralController.h"

class UnitreePolicyWrapper : public NeuralController  {
public:
    UnitreePolicyWrapper(std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device);
    CustomTypes::Action getControlAction(const CustomTypes::RobotData &robotData) override;
    
private:
    void updateObservation(const CustomTypes::RobotData& robotData);
};
