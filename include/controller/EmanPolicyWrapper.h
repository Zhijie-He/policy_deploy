#pragma once
#include "controller/BasePolicyWrapper.h"

class EmanPolicyWrapper : public BasePolicyWrapper  {
public:
    EmanPolicyWrapper(std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device);
    CustomTypes::Action getControlAction(const CustomTypes::RobotData &robotData) override;
    
private:
    void updateObservation(const CustomTypes::RobotData& robotData);
};
