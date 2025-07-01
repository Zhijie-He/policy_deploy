// tasks/BaseTask.h
#pragma once
#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <torch/torch.h>

struct BaseTaskCfg {
    virtual ~BaseTaskCfg() = default;
};

class BaseTask {
public:
     BaseTask(std::shared_ptr<BaseTaskCfg> cfg, float control_dt, torch::Device device)
        : cfg_(cfg), control_dt_(control_dt), device_(device), counter_(0), start_(false) {}
        
    virtual ~BaseTask() = default;

    virtual void resolveKeyboardInput(char key) = 0;
    virtual std::unordered_map<std::string, Eigen::MatrixXf> resolveObs(
        const Eigen::VectorXf& self_obs,
        const Eigen::VectorXf& raw_obs) = 0;

    Eigen::VectorXf getAction(const Eigen::VectorXf& self_obs, const Eigen::VectorXf& raw_obs){};
    virtual std::string getVisualization(const Eigen::VectorXf&, const Eigen::VectorXf&, const Eigen::VectorXf&) { return visualization_; }
    virtual void reset() { counter_ = 0; start_ = false; }

protected:
    std::shared_ptr<BaseTaskCfg> cfg_;
    float control_dt_;
    torch::Device device_;
    int counter_;
    bool start_;
    std::string visualization_;
};

