// tasks/MocapTask.h
#pragma once
#include <fstream>
#include "tasks/BaseTask.h"
#include "tasks/TaskFactory.h"
#include "tasks/utils/mocap/MocapMsgSubscriber.h"
#include "utility/json.hpp"

using json = nlohmann::json;

struct MocapTaskCfg : public BaseTaskCfg {
    std::string policy_path = std::string(PROJECT_SOURCE_DIR) + "/resources/policies/g1/teleopTask.pt";
    std::string engine_path = std::string(PROJECT_SOURCE_DIR) + "/resources/policies/g1/teleopTask.engine";

    // 模型参数
    int num_obs = 574;                       //   94 + 90 + 4x90 + 30 
    int num_hidden = 5704;                    //  5704  = 31 x (94 + 90)
    int num_actions = 29;

    int num_samples = 1 + 4;          // 1 current + 4 next
    float sample_timestep_inv = 30.0f;

    std::vector<std::string> track_keypoints_names = {
        "pelvis",
        "left_knee_link", "left_ankle_roll_link", "right_knee_link", "right_ankle_roll_link",
        "left_elbow_link","left_wrist_yaw_link", "right_elbow_link","right_wrist_yaw_link"
    };

    std::vector<std::string> BODY_NAMES = {
        "pelvis",
        "left_hip_pitch_link","right_hip_pitch_link","waist_yaw_link",
        "left_hip_roll_link", "right_hip_roll_link", "waist_roll_link",
        "left_hip_yaw_link", "right_hip_yaw_link", "torso_link",
        "left_knee_link", "right_knee_link", "left_shoulder_pitch_link", "right_shoulder_pitch_link",
        "left_ankle_pitch_link", "right_ankle_pitch_link", "left_shoulder_roll_link", "right_shoulder_roll_link",
        "left_ankle_roll_link", "right_ankle_roll_link", "left_shoulder_yaw_link", "right_shoulder_yaw_link",
        "left_elbow_link", "right_elbow_link",
        "left_wrist_roll_link", "right_wrist_roll_link",
        "left_wrist_pitch_link", "right_wrist_pitch_link",
        "left_wrist_yaw_link", "right_wrist_yaw_link"
    };

    std::string getPolicyPath() const override{
        return policy_path;
    }

    std::string getEnginePath() const override{
        return engine_path;
    }
    
    int getNumActions() const override{
        return num_actions;
    }

    int getNumObs() const override{
        return num_obs;
    }

    int getNumHidden() const override{
        return num_hidden;
    }
};

class MocapTask : public BaseTask {
public:
    MocapTask(std::shared_ptr<const BaseRobotConfig> cfg,
              torch::Device device,
              const std::string& hands_type,
              const std::string& inference_engine_type,
              const std::string& precision);
    CustomTypes::Action getAction(const CustomTypes::RobotData &robotData) override;
    void resolveKeyboardInput(char key, CustomTypes::RobotData &robotData) override;
    void resolveSelfObservation(const CustomTypes::RobotData& robotData) override;
    void resolveTaskObservation(const CustomTypes::RobotData& robotData) override;
    void reset() override;
    MocapResult getMocap();
    CustomTypes::Action getHandsAction(CustomTypes::Action& robotAction);

private:
    MocapTaskCfg task_cfg_;
    Eigen::VectorXi track_keypoints_indices_;
    Eigen::VectorXf mask_;
    std::unique_ptr<MocapMsgSubscriber> mocap_receiver_;
    Eigen::VectorXi left_wrist_ids_, right_wrist_ids_;
};


// Register Task
namespace {
bool registered = []() {
    TaskFactory::registerTask("MocapTask", 
        [](std::shared_ptr<const BaseRobotConfig> cfg, 
           torch::Device device,
           const std::string& hands_type,
           const std::string& inference_engine_type,
           const std::string& precision) {
            return std::make_shared<MocapTask>(cfg, device, hands_type, inference_engine_type, precision);
        });
    return true;
}();
}