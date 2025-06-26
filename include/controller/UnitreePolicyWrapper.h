#pragma once
#include "controller/BasePolicyWrapper.h"

class UnitreePolicyWrapper : public BasePolicyWrapper  {
public:
    UnitreePolicyWrapper(std::shared_ptr<const BaseRobotConfig> cfg, torch::Device device);
    CustomTypes::Action getControlAction(const CustomTypes::RobotData &robotData) override;
    
private:
    void updateObservation(const CustomTypes::RobotData& robotData);
};
