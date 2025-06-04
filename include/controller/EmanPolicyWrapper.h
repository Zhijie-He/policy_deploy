#pragma once
#include "controller/NeuralController.h"

class EmanPolicyWrapper : public NeuralController  {
public:
    EmanPolicyWrapper(std::shared_ptr<const BaseRobotConfig> cfg);
    CustomTypes::Action getControlAction(const CustomTypes::RobotData &robotData) override;
    
private:
    void updateObservation(const CustomTypes::RobotData& robotData);
};
