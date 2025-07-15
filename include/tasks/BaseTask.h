// tasks/BaseTask.h
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <torch/torch.h>
#include "types/CustomTypes.h"
#include "config/BaseRobotConfig.h" 
// #include "inference_engine/BasePolicyInferenceEngine.h"

class BasePolicyInferenceEngine;

struct BaseTaskCfg {

    float action_clip = 10.0;
    float action_scale = 0.25;
    std::unordered_map<std::string, float> self_obs_scale = {
        {"heading", 0.5f},
        {"projected_gravity_b", 1.0f},
        {"root_ang_vel_b", 0.25f},
        {"joint_pos", 1.0f},
        {"joint_vel", 0.05f},
        {"action", 1.0f}
    };

    virtual ~BaseTaskCfg() = default;

    virtual std::string getPolicyPath() const = 0;
    virtual std::string getEnginePath() const = 0;
    virtual int getNumActions() const = 0;
    virtual int getNumObs() const = 0;
    virtual int getNumHidden() const = 0;
};

class BaseTask {
public:
     BaseTask(std::shared_ptr<const BaseRobotConfig> cfg,
              std::shared_ptr<const BaseTaskCfg> task_cfg, 
              torch::Device device,
              const std::string& hands_type,
              const std::string& inference_engine_type,
              const std::string& precision);
              
    virtual ~BaseTask() = default;
    virtual CustomTypes::Action getAction(const CustomTypes::RobotData &robotData);
    
    virtual void resolveObservation(const CustomTypes::RobotData& robotData);
    virtual void resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) = 0;
    virtual void reset();
    const Eigen::MatrixXf& getVisualization() const;
    void setVisualization(const std::vector<std::array<float, 3>>& vis);
    
protected:
    virtual void resolveSelfObservation(const CustomTypes::RobotData& robotData);
    virtual void resolveTaskObservation(const CustomTypes::RobotData& robotData) = 0;

    int obDim, acDim, handsDim;
    std::string hands_type_;
    std::shared_ptr<const BaseRobotConfig> cfg_;
    std::shared_ptr<const BaseTaskCfg> task_cfg_;
    int counter_;
    bool start_;

    Eigen::VectorXf _kP;
    Eigen::VectorXf _kD;
    Eigen::VectorXf observation;
    Eigen::VectorXf action, actionPrev;
    std::shared_ptr<BasePolicyInferenceEngine> engine_;

    Eigen::MatrixXf visualization_;
};

