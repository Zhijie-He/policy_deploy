// tasks/BaseTask.h
#pragma once
#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <torch/torch.h>

class BaseTask {
public:
    BaseTask(float dt, torch::Device device) : dt_(dt), device_(device), counter_(0), start_(false) {}
    virtual ~BaseTask() = default;

    virtual void resolveKeyboardInput(char key) = 0;
    virtual std::unordered_map<std::string, Eigen::MatrixXf> resolveObs(
        const Eigen::VectorXf& self_obs,
        const Eigen::VectorXf& raw_obs) = 0;

    virtual Eigen::VectorXf getAction(const Eigen::VectorXf& self_obs, const Eigen::VectorXf& raw_obs) = 0;
    virtual std::string getVisualization(const Eigen::VectorXf&, const Eigen::VectorXf&, const Eigen::VectorXf&) { return visualization_; }
    virtual void reset() { counter_ = 0; start_ = false; }

protected:
    float dt_;
    torch::Device device_;
    int counter_;
    bool start_;
    std::string visualization_;
};

